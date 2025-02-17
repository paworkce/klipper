// Delta kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

enum stepper_type {
    STEPPER_DELTA,
    STEPPER_R,
    STEPPER_T
};

struct delta_stepper {
    struct stepper_kinematics sk;
    double arm2, tower_x, tower_y;
    enum stepper_type type;
};

static double
delta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    
    switch (ds->type) {
        case STEPPER_R:
            return c.r;
        case STEPPER_T:
            return c.t;
        default:
            double dx = ds->tower_x - c.x, dy = ds->tower_y - c.y;
            return sqrt(ds->arm2 - dx*dx - dy*dy) + c.z;
    }
}

struct stepper_kinematics * __visible
delta_stepper_alloc(double arm2, double tower_x, double tower_y, enum stepper_type type)
{
    struct delta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->arm2 = arm2;
    ds->tower_x = tower_x;
    ds->tower_y = tower_y;
    ds->type = type;
    ds->sk.calc_position_cb = delta_stepper_calc_position;
    
    // Set active flags based on stepper type
    switch (type) {
        case STEPPER_R:
            ds->sk.active_flags = AF_R;
            break;
        case STEPPER_T:
            ds->sk.active_flags = AF_T;
            break;
        default:
            ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    }
    
    return &ds->sk;
}

struct rt_stepper {
    struct stepper_kinematics sk;
    double axis_coord;
};

static double
rt_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct rt_stepper *rs = container_of(sk, struct rt_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    if (rs->sk.active_flags == AF_R)
        return c.r;
    return c.t;
}

struct stepper_kinematics * __visible
rt_stepper_alloc(double axis_coord, int axis_is_r)
{
    struct rt_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->axis_coord = axis_coord;
    rs->sk.active_flags = axis_is_r ? AF_R : AF_T;
    rs->sk.calc_position_cb = rt_stepper_calc_position;
    return &rs->sk;
}
