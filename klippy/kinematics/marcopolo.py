# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes
import math

class MarcKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzrt']
        for rail, axis in zip(self.rails, 'xyzrt'):
            rail.setup_itersolve('marcopolo_stepper_alloc', axis.encode())
        # Add rail for the "r" axis
        # r_config = config.getsection('stepper_r')
        # self.rails.append(stepper.LookupMultiRail(r_config))
        # self.rails[-1].setup_itersolve('marcopolo_stepper_alloc', 'r'.encode())
        ranges = [r.get_range() for r in self.rails[:5]]  # use only x, y, z for original ranges
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        self.dc_module = None
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[5].setup_itersolve('marcopolo_stepper_alloc',
                                          dc_axis.encode())
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[3], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        # Adjust limits for five axes (changed from 4 axes)
        self.limits = [(1.0, -1.0)] * 5
        # Read rotation center (default 0,0)
        self.rotation_origin = (
            config.getfloat('rotation_origin_x', 0),
            config.getfloat('rotation_origin_y', 0)
        )
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        rails = self.rails
        if self.dc_module:
            primary_rail = self.dc_module.get_primary_rail().get_rail()
            rails = (rails[:self.dc_module.axis] +
                     [primary_rail] + rails[self.dc_module.axis+1:])
        # Get raw position for each axis
        pos = [stepper_positions[rail.get_name()] for rail in rails]
        # Apply rotation transformation to "x" and "y"
        raw_x = stepper_positions['x']
        raw_y = stepper_positions['y']
        theta = stepper_positions['r']
        x0, y0 = self.rotation_origin
        new_x = (raw_x - x0) * math.cos(theta) - (raw_y - y0) * math.sin(theta) + x0
        new_y = (raw_x - x0) * math.sin(theta) + (raw_y - y0) * math.cos(theta) + y0
        # Update positions for x and y
        for idx, rail in enumerate(self.rails):
            if rail.get_name() == 'x':
                pos[idx] = new_x
            if rail.get_name() == 'y':
                pos[idx] = new_y
        return pos
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = "xyzrt".index(axis_name)
            if self.dc_module and axis == self.dc_module.axis:
                rail = self.dc_module.get_primary_rail().get_rail()
            else:
                rail = self.rails[axis]
            self.limits[axis] = rail.get_range()
    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyzrt"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                # Use the correct index from "xyzr"
                idx = "xyzrt".index(axis)
                self.home_axis(homing_state, idx, self.rails[idx])
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzrt", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    try:
        return MarcKinematics(toolhead, config)
    except Exception as e:
        raise config.error("Error loading kinematics 'marcopolo': {}".format(e))
