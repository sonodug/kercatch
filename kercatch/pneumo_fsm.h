#ifndef PNEUMO_CTRL_H
#define PNEUMO_CTRL_H

#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

    enum State
    {
        PneumoState_1 = 0,
        PneumoState_2,
        PneumoState_3,
        PneumoState_4,
        PneumoState_5,
        PneumoState_6,
        PneumoState_7,
        PneumoState_8,
        PneumoState_9,
        PneumoState_10,
        PneumoState_11,
        PneumoState_12,
        PneumoState_13,
        PneumoState_14,
        PneumoState_15,
        PneumoState_16,
        PneumoState_17,
        PneumoState_18,
        PneumoState_Exception
    };

#define SIGNAL_MAX  0
#define SIGNAL_MIN  1

    struct CylinderIO
    {
        int inputSignals[2];
        int outputSignal;
    };

#define PNEUMOCYL_Y1 0
#define PNEUMOCYL_Y2 1
#define PNEUMOCYL_Y3 2
#define PNEUMOCYL_Y4 3
#define PNEUMOCYL_Y5 4
#define PNEUMOCYL_Y6 5
#define PNEUMOCYL_Y7 6
#define PNEUMOCYL_Y8 7

    struct Machine
    {
        enum State state;
        int exceptionInputSignal;
        int timeout;
        int delay;
        int timeouts[PneumoState_Exception];
        int delays[PneumoState_Exception];
        struct CylinderIO cylinders[8];
    };

    void pneumocyl_machine_init(struct Machine* machine);

    bool pneumocyl_machine_tick(struct Machine* machine);

    void pneumocyl_machine_destroy(struct Machine* machine);

    void reset_signals(struct Machine* machine);

    void reset_output_signals(struct Machine* machine);

    void reset_time_params(struct Machine* machine);

    void set_params(struct Machine* machine, int* timeoutDel, int* delayDel);

#if defined(__cplusplus)
}
#endif

#endif