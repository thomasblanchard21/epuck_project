#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <gate_detection.h>
#include <leds.h>
#include <sensors/proximity.h>

static THD_WORKING_AREA(waGateDetection, 256);
static THD_FUNCTION(GateDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    proximity_start();

    calibrate_ir();
    //Attend un peu apres calibration
    set_led(1,1);
    chThdSleepMilliseconds(2000);
    set_led(1,0);


    systime_t time;

    static uint8_t nb_leds=0;

    while(1){

        time = chVTGetSystemTime();


        if ((get_calibrated_prox(2) > 0) && (get_calibrated_prox(5) > 0)) {
        	++nb_leds;

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

        	chThdSleepMilliseconds(2000);
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void gate_detection_start(void){
	chThdCreateStatic(waGateDetection, sizeof(waGateDetection), NORMALPRIO, GateDetection, NULL);
}
