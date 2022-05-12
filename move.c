#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <move.h>

#include <sensors/imu.h>

void refresh_position(imu_msg_t *imu_values){


    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 1;
    float threshold_inf = 0.6;
    float threshold_back = 0.4;

    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    float speed = 1000;
    float acc_coeff = 1000;

 //   chSysLock();

    if (accel[X_AXIS] > threshold ) {
      	speed=abs((accel[X_AXIS]-threshold+1)*acc_coeff);
       	left_motor_set_speed(-speed);
       	right_motor_set_speed(speed);
    } else if(accel[X_AXIS] < -threshold) {
      	speed=abs((accel[X_AXIS]+threshold-1)*acc_coeff);
       	left_motor_set_speed(speed);
       	right_motor_set_speed(-speed);
    } else if ( (accel[Y_AXIS] < -threshold) && (abs(accel[X_AXIS]) < threshold_inf) ) {
          	speed=abs((accel[Y_AXIS]+threshold-1)*acc_coeff);
         	left_motor_set_speed(speed);
           	right_motor_set_speed(speed);
    } else if ( (accel[Y_AXIS] > threshold) && (abs(accel[X_AXIS]) < threshold_back) ) {
      	speed=abs((accel[Y_AXIS]-threshold+1)*acc_coeff);
       	left_motor_set_speed(-speed);
    	right_motor_set_speed(speed);
    } else if ((abs(accel[X_AXIS]) < threshold_inf ) && (abs(accel[Y_AXIS]) < threshold_inf )) {
       	left_motor_set_speed(0);
       	right_motor_set_speed(0);
    }

//   chSysUnlock();
}



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

