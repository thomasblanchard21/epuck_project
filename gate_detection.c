#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <spi_comm.h>
#include <gate_detection.h>
#include <leds.h>
#include <sensors/proximity.h>

static THD_WORKING_AREA(waGateDetection, 256);
static THD_FUNCTION(GateDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Start the proximity sensors
    proximity_start();

    //start RGB LEDs
//	spi_comm_start();

    systime_t time;

    //inDoor true if robot is in a gate, else false
    static bool inDoor=1;
    static uint8_t nb_leds=0;


    //Visual signal that calibration is in progress

    for (int i=0; i<4; ++i) {
    	set_led(i,1);
    	chThdSleepMilliseconds(100);
//    	set_rgb_led(i,0,0,0);
//    	chThdSleepMilliseconds(100);
    }

    calibrate_ir();

    //Choose shortest distance for calibration (see calibration procedure)
    uint16_t calibration=0;
    if (get_prox(2) > get_prox(5)) {
    	calibration=get_prox(2);
    } else {
    	calibration=get_prox(5);
    }

    for (int i=0; i<4; ++i) {
    	set_led(i,0);
    	chThdSleepMilliseconds(100);
//    	set_rgb_led(i,0,0,0);
//    	chThdSleepMilliseconds(100);
    }

    while(1){

        time = chVTGetSystemTime();

        //Robot is in a gate?
        if ((get_calibrated_prox(2)/calibration > 0) && (get_calibrated_prox(5)/calibration > 0)) {
        	if (inDoor==0) {
        		++nb_leds;
        	}
    		inDoor=1;

        	set_body_led(1);

        	//FSM for the LEDs
            switch(nb_leds%6) {
            		case 0:
            			clear_leds();
            			break;
            		case 1:
            			set_led(0,1);
            			break;
            		case 2:
            		    set_led(1,1);
            		    break;
            		case 3:
            			set_led(2,1);
            			break;
            		case 4:
            		    set_led(3,1);
            		    break;
            		case 5:
            		    for(int i=0; i<11; i++) {
            		    	for(int j=0; j<4; j++) {
            		    		set_led(j,2);
            		    	}
        		    	chThdSleepMilliseconds(200);
            		    }
            		    break;
            		default:
            			clear_leds();
            }
        } else {
        	inDoor=0;
        	set_body_led(0);
        }
        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }


}


void gate_detection_start(void){
	chThdCreateStatic(waGateDetection, sizeof(waGateDetection), NORMALPRIO, GateDetection, NULL);
}
