#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Starts the serial communication
    serial_start();

    i2c_start();
    imu_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	//inits the motors
	motors_init();


    //wait 2 sec to be sure the e-puck is in a stable position
	set_led(0,1);
    chThdSleepMilliseconds(2000);
    calibrate_acc();
	set_led(0,0);


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
