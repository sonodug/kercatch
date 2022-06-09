#include <stdio.h>
#include <limits.h>
#include "pneumo_fsm.h"

#define TOUT_DEL(timeout)  ((timeout) * 1000)
#define D_DEL(delay)      ((delay) * 1000)

void pneumocyl_machine_init(struct Machine* machine)
{
    if (machine)
    {
        reset_signals(machine);

        machine->state = PneumoState_1;
        reset_time_params(machine);

        int timeout_deltas[] = { TOUT_DEL(45), TOUT_DEL(60), TOUT_DEL(120), TOUT_DEL(45),
        TOUT_DEL(56), TOUT_DEL(60), TOUT_DEL(30), TOUT_DEL(120), TOUT_DEL(56), TOUT_DEL(30), TOUT_DEL(60), 
        TOUT_DEL(30), TOUT_DEL(60), TOUT_DEL(45), TOUT_DEL(120), TOUT_DEL(56), TOUT_DEL(60), TOUT_DEL(60) };

        int delay_deltas[] = { D_DEL(78), D_DEL(33), D_DEL(70), D_DEL(45), D_DEL(70),
        D_DEL(60), D_DEL(33), D_DEL(78), D_DEL(78), D_DEL(45), D_DEL(33), D_DEL(70),
        D_DEL(60), D_DEL(70), D_DEL(45), D_DEL(78), D_DEL(45), D_DEL(45) };

        set_params(machine, timeout_deltas, delay_deltas);
    }
}

#define TOUT_COMP(machine) ( (machine)->timeout > (machine)->timeouts[(machine)->state] )
#define D_COMP(machine) ( (machine)->delay > (machine)->delays[(machine)->state] )

bool pneumocyl_machine_tick(struct Machine* machine)
{
    bool execution_flag = true;

    if (machine == 0)
        return false;

    switch (machine->state)
    {
        case PneumoState_1:
        {
            reset_output_signals(machine);

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_2;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_2:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_3;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_3:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_4;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_7;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_4:
        {
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_5;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_13;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_5:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_6;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_6:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_7;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_7:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_8;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }

            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_8:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_9;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_9:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_10;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_10:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_11;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_11:
        {
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_12;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_12:
        {
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_13;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_13:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_14;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_14:
        {
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_15;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_15:
        {
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_16;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_16:
        {
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y4].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y5].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;
            machine->cylinders[PNEUMOCYL_Y7].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN] &&
                machine->cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_17;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_17:
        {
            machine->cylinders[PNEUMOCYL_Y8].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y1].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y2].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y3].outputSignal = 1;
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 1;

            if (machine->cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX] &&
                machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_18;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_18:
        {
            machine->cylinders[PNEUMOCYL_Y6].outputSignal = 0;

            if (machine->cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN])
            {
                machine->timeout = 0;

                if (D_COMP(machine) && !machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_1;
                    reset_time_params(machine);
                }
                else if (machine->exceptionInputSignal)
                {
                    machine->state = PneumoState_Exception;
                    reset_time_params(machine);
                }
            }
            else if (TOUT_COMP(machine))
            {
                machine->state = PneumoState_Exception;
                reset_time_params(machine);
            }

            break;
        }
        case PneumoState_Exception:
        {
            reset_output_signals(machine);
            pneumocyl_machine_destroy(machine);
            execution_flag = false;

            break;
        }
    }

    machine->timeout++;
    machine->delay++;
    return execution_flag;
}


void pneumocyl_machine_destroy(struct Machine* machine)
{
    if (machine != 0)
    {
        machine = 0;
    }
}

void reset_signals(struct Machine* machine)
{
    for (int i = 0; i < 8; i++)
    {
        machine->cylinders[i].inputSignals[SIGNAL_MAX] = 0;
        machine->cylinders[i].inputSignals[SIGNAL_MIN] = 0;
        machine->cylinders[i].outputSignal = 0;
    }
}

void reset_output_signals(struct Machine* machine)
{
    for (int i = 0; i < 8; i++)
    {
        machine->cylinders[i].outputSignal = 0;
    }
}

void reset_time_params(struct Machine* machine)
{
    machine->delay = 0;
    machine->timeout = 0;
}

void set_params(struct Machine* machine, int* timeout_delta, int* delay_delta)
{
    for (int i = 0; i < PneumoState_Exception; ++i)
    {
        machine->timeouts[i] = timeout_delta[i];
        machine->delays[i] = delay_delta[i];
    }
}