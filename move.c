#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <move.h>

#include <imu.h>


static THD_WORKING_AREA(waMove, 256);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;




    systime_t time;

    while(1){

        time = chVTGetSystemTime();

        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        refresh_position(&imu_values);
 

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}
