#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>

#include "DEV_Config.h"
#include "PCA9685.h"

void  Handler(int signo)
{
    // System Exit
    printf("\r\nHandler: Program stop\r\n"); 

    IIC_Write(MODE2,0x00);
    DEV_ModuleExit();

    exit(0);
}

int main(int argc, char **argv)
{
	int angle = 90;
	int range = 50;
	int inc = 1;
	int interval = 50;

	if (argc > 1) {
		range = atoi(argv[1]);
		if (range > 90) {
			range = 90;
		} else if (range < 0) {
			range = 0;
		}
	}

	if (argc > 2) {
		inc = atoi(argv[2]);
	}

	if (argc > 3) {
		interval = atoi(argv[3]);
	}

	if (DEV_ModuleInit() == 1) {
		return 1;
	}

    // Exception handling: Ctrl+C
    signal(SIGINT, Handler);

    // PCA9685 initialization
	Init_PCA9685();

	while(1)
	{
		printf("Servo angle: %d\r\n", angle);

		PCA9685_Set_Rotation_Angle(0, angle);
		PCA9685_Set_Rotation_Angle(1, angle);

		DEV_Delay_ms(interval);

		angle += inc;
		if ((angle > (90 + range)) || (angle < (90 - range))) {
			inc = -inc;
			angle += inc;
		}
	}

	DEV_ModuleExit();
    return 0;
}
