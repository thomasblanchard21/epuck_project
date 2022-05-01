#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <sensors/imu.h>

#include <gate_detection.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
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

}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    i2c_start();
    imu_start();

    //Starts the serial communication
    serial_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	//inits the motors
	motors_init();

	//Inits the proximity sensors


	//Inits the gate detection
	gate_detection_start();

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    calibrate_acc();

    /* Infinite loop. */
    while (1) {

    	//wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        refresh_position(&imu_values);
    	//waits 1 second
        chThdSleepMilliseconds(10);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
