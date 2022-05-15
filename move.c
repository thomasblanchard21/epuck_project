#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <move.h>
#include <sensors/imu.h>

void refresh_position(imu_msg_t *imu_values){

    //Threshold values to not move when the robot is too horizontal
    float threshold_sup = 1.2;
    float threshold_inf = 1.0;

    //Create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    //Speed of the left and right motors and a value to store the new speed to add
    float speed_r = 0;
    float speed_l = 0;
    float speed = 0;

    //Acceleration coefficient to correlate the speed of the robot to the tilt
    float acc_coeff = 500;

    if (accel[X_AXIS] > threshold_sup ) {
      	speed=abs(accel[X_AXIS]*acc_coeff);
      	speed_r=speed_r+speed;
      	speed_l=speed_l-speed;
    }
    if(accel[X_AXIS] < -threshold_sup) {
      	speed=abs(accel[X_AXIS]*acc_coeff);
      	speed_r=speed_r-speed;
      	speed_l=speed_l+speed;
    }
    if (accel[Y_AXIS] < -threshold_sup) {
        speed=abs(accel[Y_AXIS]*acc_coeff);
        speed_r=speed_r+speed;
        speed_l=speed_l+speed;
    }
    if (accel[Y_AXIS] > threshold_sup) {
        speed=abs(accel[Y_AXIS]*acc_coeff);
        speed_r=speed_r-speed;
        speed_l=speed_l-speed;
    }
    if ((abs(accel[X_AXIS]) < threshold_inf ) && (abs(accel[Y_AXIS]) < threshold_inf )) {
       	speed_r=0;
       	speed_l=0;
    }

    left_motor_set_speed(speed_l);
    right_motor_set_speed(speed_r);

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

        //Wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        refresh_position(&imu_values);

        //Frequency of 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

