// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include <math.h>   // added for sqrt
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

static double
marcopolo_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
marcopolo_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
marcopolo_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    return move_get_coord(m, move_time).z;
}

static double
marcopolo_stepper_r_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    return move_get_coord(m, move_time).r;
}

static double
marcopolo_stepper_t_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    return move_get_coord(m, move_time).t;
}

struct stepper_kinematics * __visible
marcopolo_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x') {
        sk->calc_position_cb = marcopolo_stepper_x_calc_position;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = marcopolo_stepper_y_calc_position;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = marcopolo_stepper_z_calc_position;
        sk->active_flags = AF_Z;
    } else if (axis == 'r') {
        sk->calc_position_cb = marcopolo_stepper_r_calc_position;
        sk->active_flags = AF_R; // assume AF_R is defined elsewhere
    } else if (axis == 't') {
        sk->calc_position_cb = marcopolo_stepper_t_calc_position;
        sk->active_flags = AF_T; // assume AF_T is defined elsewhere
    }
    return sk;
}
