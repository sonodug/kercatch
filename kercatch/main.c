#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pneumo_fsm.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "Rus");

    FILE* fd;

    if (argc < 2)
    {
        fprintf(stderr, "Не передан файл симуляции\n");
        return EXIT_FAILURE;
    }

    fprintf(stdout, "Файл симуляции передан: \"%s\"\n", argv[1]);
    fd = fopen(argv[1], "rt");

    if (fd)
    {
        struct Machine engine;
        bool running = true;

        pneumocyl_machine_init(&engine);
        while (running)
        {
            int eq_output[8];

            if (feof(fd))
            {
                fprintf(stdout, "Завершение файла симуляции\n");
                running = false;
                continue;
            }
            
            fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                (int*)&engine.cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y1].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y2].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y3].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y4].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y5].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y6].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y7].inputSignals[SIGNAL_MIN],
                (int*)&engine.cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MAX],
                (int*)&engine.cylinders[PNEUMOCYL_Y8].inputSignals[SIGNAL_MIN],
                (int*)&eq_output[PNEUMOCYL_Y1],
                (int*)&eq_output[PNEUMOCYL_Y2],
                (int*)&eq_output[PNEUMOCYL_Y3],
                (int*)&eq_output[PNEUMOCYL_Y4],
                (int*)&eq_output[PNEUMOCYL_Y5],
                (int*)&eq_output[PNEUMOCYL_Y6],
                (int*)&eq_output[PNEUMOCYL_Y7],
                (int*)&eq_output[PNEUMOCYL_Y8]
            );
            running = pneumocyl_machine_tick(&engine);
           
            if (!running)
            {
                fprintf(stderr, "Остановка из-за критической ошибки автомата\n");
            }
        }

        pneumocyl_machine_destroy(&engine);
    }
    if (0 != fd)
    {
        fclose(fd);
    }
    return EXIT_SUCCESS;
}