#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <move.h>

#include <sensors/imu.h>


static THD_WORKING_AREA(waMove, 256);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

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

void refresh_position(imu_msg_t *imu_values){


    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 2.0;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    float speed = 1000;

//    chSysLock();

    if(accel[X_AXIS] > threshold) {
    	left_motor_set_speed(-speed);
    	right_motor_set_speed(speed);
    } else if(accel[X_AXIS] < -threshold) {
    	left_motor_set_speed(speed);
    	right_motor_set_speed(-speed);
    } else if(accel[Y_AXIS] > threshold) {
    	left_motor_set_speed(-speed);
    	right_motor_set_speed(speed);
    } else if(accel[Y_AXIS] < -threshold) {
    	left_motor_set_speed(speed);
    	right_motor_set_speed(speed);
    } else {
    	left_motor_set_speed(0);
    	right_motor_set_speed(0);
    }

    //phase a 0 ?

//    chSysUnlock();

    //led
}
