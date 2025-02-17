#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

struct coord {
    union {
        struct {
            double x, y, z, r ,t;
        };
        double axis[5];
    };
};

struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;

    struct list_node node;
};

struct trapq {
    struct list_head moves, history;
};

struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_x, start_y, start_z, start_r, start_t;
    double x_r, y_r, z_r, r_r, t_r;
};

struct move *move_alloc(void);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z, double start_pos_r, double start_pos_t
                  , double axes_r_x, double axes_r_y, double axes_r_z, double axes_r_r, double axes_r_t
                  , double start_v, double cruise_v, double accel);
void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , double pos_x, double pos_y, double pos_z, double pos_r, double pos_t);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
