#include <stdio.h>
#include <limits.h>
#include "pneumo_fsm.h"

#define TOUT_DEL(timeout)  ((timeout) * 100)
#define D_DEL(delay)      ((delay) * 100)

#define TOUT_COMP(machine) ( (machine)->timeout > (machine)->timeouts[(machine)->state] )
#define D_COMP(machine) ( (machine)->delay > (machine)->delays[(machine)->state] )

void pneumocyl_machine_init(struct Machine* machine) {
    if (machine) {
        reset_signals(machine);

        machine->state = State_0;
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

bool pneumocyl_machine_tick(struct Machine* machine) {
    bool execution_flag = true;

    if (machine == 0)
        return false;

    switch (machine->state) {
    case State_0: {
        reset_output_signals(machine);

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_1;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_1: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX]) {

            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_2;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_2: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {
           
            machine->timeout = 0;
            machine->delay++;
            
            if (D_COMP(machine)) {
                machine->state = State_3;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_3: {
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_8;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_7;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_7;
            reset_time_params(machine);
        }

        break;
    }
    case State_4: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX]) {

            machine->timeout = 0;

            if (D_COMP(machine)) {
                machine->state = State_5;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_13;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_13;
            reset_time_params(machine);
        }

        break;
    }
    case State_5: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {

            machine->timeout = 0;

            if (D_COMP(machine)) {
                machine->state = State_6;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_6: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX]) {

            machine->timeout = 0;

            if (D_COMP(machine)) {
                machine->state = State_7;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_7: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX]) {

            machine->timeout = 0;

            if (D_COMP(machine)) {
                machine->state = State_8;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_8: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {
           
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_9;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_9: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX]) {
           
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_10;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_10: {
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_11;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_11: {
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MAX]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_12;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_12: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_13;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_13: {
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MIN]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_14;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_14: {
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MIN]) {

            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_15;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_15: {
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y4].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y5].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;
        machine->cylinders[PNEUMOCYL_Y7].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y4].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y5].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN] &&
            machine->cylinders[PNEUMOCYL_Y7].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MIN]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_16;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_16: {
        machine->cylinders[PNEUMOCYL_Y8].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y1].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y2].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y3].output_signal = 1;
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 1;

        if (machine->cylinders[PNEUMOCYL_Y8].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y1].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y2].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y3].input_signals[SIGNAL_MAX] &&
            machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MAX]) {
            
            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_17;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_17: {
        machine->cylinders[PNEUMOCYL_Y6].output_signal = 0;

        if (machine->cylinders[PNEUMOCYL_Y6].input_signals[SIGNAL_MIN]) {

            machine->timeout = 0;
            machine->delay++;

            if (D_COMP(machine)) {
                machine->state = State_0;
                reset_time_params(machine);
            }
        }
        else if (TOUT_COMP(machine)) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }
        if (machine->exception_input_signal) {
            machine->state = State_Exception;
            reset_time_params(machine);
        }

        break;
    }
    case State_Exception: {
        reset_output_signals(machine);
        pneumocyl_machine_destroy(machine);
        execution_flag = false;

        break;
    }
    }

    machine->timeout++;
    return execution_flag;
}


void pneumocyl_machine_destroy(struct Machine* machine) {
    if (machine != 0) {
        machine = 0;
    }
}

void reset_signals(struct Machine* machine) {
    for (int i = 0; i < 8; i++) {
        machine->cylinders[i].input_signals[SIGNAL_MAX] = 0;
        machine->cylinders[i].input_signals[SIGNAL_MIN] = 0;
        machine->cylinders[i].output_signal = 0;
    }
}

void reset_output_signals(struct Machine* machine) {
    for (int i = 0; i < 8; i++) {
        machine->cylinders[i].output_signal = 0;
    }
}

void reset_time_params(struct Machine* machine) {
    machine->delay = 0;
    machine->timeout = 0;
}

void set_params(struct Machine* machine, int* timeout_delta, int* delay_delta) {
    for (int i = 0; i < State_Exception; ++i) {
        machine->timeouts[i] = timeout_delta[i];
        machine->delays[i] = delay_delta[i];
    }
}