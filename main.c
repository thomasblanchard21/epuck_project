#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <sensors/imu.h>
#include <gate_detection.h>
#include <move.h>
#include <leds.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{

    //Multiple initialisations
	halInit();
    chSysInit();
    mpu_init();

    //Start the imu
    imu_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	//Inits the motors
	motors_init();

	//Wait 2 sec to be sure the e-puck is in a stable position before calibrating
	//the accelerometer while turning on the body led
	set_body_led(1);
    chThdSleepMilliseconds(2000);
    calibrate_acc();
	set_body_led(0);

	//Inits the gate detection
	gate_detection_start();

    //Inits movement
	move_start();

    /* Infinite loop. */
    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
