#ifndef PNEUMO_CTRL_H
#define PNEUMO_CTRL_H

#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

    enum PneumoState
    {
        PneumoState_Init = 0,
        PneumoState_1,
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

#define PNEUMO_CYLINDER_SIGNAL_UP     0
#define PNEUMO_CYLINDER_SIGNAL_DOWN   1

    static int exceptionOutputSignal = 0;

    struct PneumoCylinder
    {
        int inputSignals[2];
        int cylinderOutputSignal;
    };

#define PNEUMOCYL_Y1 0
#define PNEUMOCYL_Y2 1
#define PNEUMOCYL_Y3 2
#define PNEUMOCYL_Y4 3
#define PNEUMOCYL_Y5 4
#define PNEUMOCYL_Y6 5
#define PNEUMOCYL_Y7 6
#define PNEUMOCYL_Y8 7

    struct PneumoEngine
    {
        enum PneumoState state;
        int timeout;
        int delay;
        int timeouts[PneumoState_Exception];
        int delays[PneumoState_Exception];
        struct PneumoCylinder cylinders[8];
    };

    void pneumocyl_engine_init(struct PneumoEngine* engine);

    bool pneumocyl_engine_tick(struct PneumoEngine* engine);

    void pneumocyl_engine_destroy(struct PneumoEngine* engine);

    void reset_signals(struct PneumoEngine* engine);

    void reset_output_signals(struct PneumoEngine* engine);

    void reset_time_params(struct PneumoEngine* engine);

    void set_params(struct PneumoEngine* engine, int* timeout_delta, int* delay_delta);

#if defined(__cplusplus)
}
#endif

#endif //PNEUMO_CTRL_H