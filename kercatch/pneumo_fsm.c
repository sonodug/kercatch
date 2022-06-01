#include <stdio.h>
#include <limits.h>
#include "pneumo_fsm.h"

//pric

//#define TIMEOUT_DELTA(timeout)  1
//#define DELAY_DELTA(delay)      1

#define TIMEOUT_DELTA(timeout)  ((timeout) * 1000)
#define DELAY_DELTA(delay)      ((delay) * 1000)

#if defined(PNEUMO_DEBUG)
static char* state_names[] =
{
        "PneumoState_Init",
        "PneumoState_1",
        "PneumoState_2",
        "PneumoState_3",
        "PneumoState_4",
        "PneumoState_5",
        "PneumoState_6",
        "PneumoState_7",
        "PneumoState_8",
        "PneumoState_9",
        "PneumoState_10",
        "PneumoState_11",
        "PneumoState_12",
        "PneumoState_13",
        "PneumoState_14",
        "PneumoState_15",
        "PneumoState_16",
        "PneumoState_17",
        "PneumoState_18",
        "PneumoState_Exception",
};
#endif

void pneumocyl_engine_init(struct PneumoEngine* engine)
{
    if (0 != engine)
    {
        reset_signals(engine);

        engine->state = PneumoState_Init;
        reset_time_params(engine);

        int timeout_deltas[] = { INT_MAX, TIMEOUT_DELTA(45), TIMEOUT_DELTA(60), TIMEOUT_DELTA(120), TIMEOUT_DELTA(45),
        TIMEOUT_DELTA(56), TIMEOUT_DELTA(60), TIMEOUT_DELTA(30), TIMEOUT_DELTA(120), TIMEOUT_DELTA(56), TIMEOUT_DELTA(30), TIMEOUT_DELTA(60), 
        TIMEOUT_DELTA(30), TIMEOUT_DELTA(60), TIMEOUT_DELTA(45), TIMEOUT_DELTA(120), TIMEOUT_DELTA(56), TIMEOUT_DELTA(60), TIMEOUT_DELTA(60) };

        int delay_deltas[] = { INT_MAX, DELAY_DELTA(78), DELAY_DELTA(33), DELAY_DELTA(70), DELAY_DELTA(45), DELAY_DELTA(70),
        DELAY_DELTA(60), DELAY_DELTA(33), DELAY_DELTA(78), DELAY_DELTA(78), DELAY_DELTA(45), DELAY_DELTA(33), DELAY_DELTA(70),
        DELAY_DELTA(60), DELAY_DELTA(70), DELAY_DELTA(45), DELAY_DELTA(78), DELAY_DELTA(45), DELAY_DELTA(45) };

        set_params(engine, timeout_deltas, delay_deltas);
    }
}

#define TIMEOUT_GE(engine) ( (engine)->timeout > (engine)->timeouts[(engine)->state] )
#define DELAY_GE(engine) ( (engine)->delay > (engine)->delays[(engine)->state] )

bool pneumocyl_engine_tick(struct PneumoEngine* engine)
{
    bool execution_flag = true;

    if (engine == 0)
        return false;

#if defined(PNEUMO_DEBUG)
    fprintf(stdout, "State: %s, Y1(in): [%d, %d], Y2(in): [%d, %d], Y3(in): [%d, %d], Y4(in): [%d, %d], Y5(in): [%d, %d], Y6(in): [%d, %d], Y7(in): [%d, %d], Y8(in): [%d, %d], Y1(out): [%d], Y2(out): [%d], Y3(out): [%d], Y4(out): [%d], Y5(out): [%d], Y6(out): [%d], Y7(out): [%d], Y8(out): [%d]\n",
        state_names[engine->state],
        engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN],
        engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP],
        engine->cylinders[PNEUMOCYL_Y1].outputSignal,
        engine->cylinders[PNEUMOCYL_Y2].outputSignal,
        engine->cylinders[PNEUMOCYL_Y3].outputSignal,
        engine->cylinders[PNEUMOCYL_Y4].outputSignal,
        engine->cylinders[PNEUMOCYL_Y5].outputSignal,
        engine->cylinders[PNEUMOCYL_Y6].outputSignal,
        engine->cylinders[PNEUMOCYL_Y7].outputSignal,
        engine->cylinders[PNEUMOCYL_Y8].outputSignal
    );
    fflush(stdout);
#endif

    switch (engine->state)
    {
        case PneumoState_Init:
        {
            engine->state = PneumoState_1;
            reset_time_params(engine);
            break;
        }
        case PneumoState_1:
        {
            reset_output_signals(engine);

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_2;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_2:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_3;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_3:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_4;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_7;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_4:
        {
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_4;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_13;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_5:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_6;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_6:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_7;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_7:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_8;
                    reset_time_params(engine);
                }
            }

            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_8:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_9;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_9:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_10;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_10:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_11;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_11:
        {
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_12;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_12:
        {
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_13;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_13:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_14;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_14:
        {
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_15;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_15:
        {
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_16;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_16:
        {
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            engine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y4].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y5].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] &&
                engine->cylinders[PNEUMOCYL_Y7].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_17;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_17:
        {
            engine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (engine->cylinders[PNEUMOCYL_Y8].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y1].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y2].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y3].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] &&
                engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_18;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_18:
        {
            engine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;

            if (engine->cylinders[PNEUMOCYL_Y6].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN])
            {
                engine->timeout = 0;

                if (DELAY_GE(engine))
                {
                    engine->state = PneumoState_1;
                    reset_time_params(engine);
                }
            }
            else if (TIMEOUT_GE(engine))
            {
                engine->state = PneumoState_Exception;
                reset_time_params(engine);
            }

            break;
        }
        case PneumoState_Exception:
        {
            reset_output_signals(engine);
            execution_flag = false;

            break;
        }
    }

    engine->timeout++;
    engine->delay++;
    return execution_flag;
}


void pneumocyl_engine_destroy(struct PneumoEngine* engine)
{
    if (engine != 0)
    {
        //Release resources
    }
}

void reset_signals(struct PneumoEngine* engine)
{
    for (int i = 0; i < 8; i++)
    {
        engine->cylinders[i].inputSignals[PNEUMO_CYLINDER_SIGNAL_UP] = 0;
        engine->cylinders[i].inputSignals[PNEUMO_CYLINDER_SIGNAL_DOWN] = 0;
        engine->cylinders[i].outputSignal = 0;
    }
}

void reset_output_signals(struct PneumoEngine* engine)
{
    for (int i = 0; i < 8; i++)
    {
        engine->cylinders[i].outputSignal = 0;
    }
}

void reset_time_params(struct PneumoEngine* engine)
{
    engine->delay = 0;
    engine->timeout = 0;
}

void set_params(struct PneumoEngine* engine, int* timeout_delta, int* delay_delta)
{
    for (int i = 0; i < PneumoState_Exception; ++i)
    {
        engine->timeouts[i] = timeout_delta[i];
        engine->delays[i] = delay_delta[i];
    }
}