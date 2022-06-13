#ifndef PNEUMO_FSM_H
#define PNEUMO_FSM_H

#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

#define PNEUMOCYL_Y1 0
#define PNEUMOCYL_Y2 1
#define PNEUMOCYL_Y3 2
#define PNEUMOCYL_Y4 3
#define PNEUMOCYL_Y5 4
#define PNEUMOCYL_Y6 5
#define PNEUMOCYL_Y7 6
#define PNEUMOCYL_Y8 7

#define SIGNAL_MAX  0
#define SIGNAL_MIN  1

    enum State {
        State_0,
        State_1,
        State_2,
        State_3,
        State_4,
        State_5,
        State_6,
        State_7,
        State_8,
        State_9,
        State_10,
        State_11,
        State_12,
        State_13,
        State_14,
        State_15,
        State_16,
        State_17,
        State_Exception,
    };

    struct Cylinder {
        int input_signals[2];
        int output_signal;
    };

    struct Machine {
        enum State state;
        int exception_input_signal;
        int timeout;
        int delay;
        int timeouts[State_Exception];
        int delays[State_Exception];
        struct Cylinder cylinders[8];
    };

    void pneumocyl_machine_init(struct Machine* machine);

    bool pneumocyl_machine_tick(struct Machine* machine);

    void pneumocyl_machine_destroy(struct Machine* machine);

    void reset_signals(struct Machine* machine);

    void reset_output_signals(struct Machine* machine);

    void reset_time_params(struct Machine* machine);

    void set_params(struct Machine* machine, int* timeout_del, int* delay_del);

#if defined(__cplusplus)
}
#endif

#endif