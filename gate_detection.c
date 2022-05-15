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




    set_rgb_led(LED2,100,0,0);
    set_rgb_led(1,100,0,0);
    set_rgb_led(2,100,0,0);
    set_rgb_led(3,100,0,0);


    systime_t time;

    static bool inDoor=1;
    static uint8_t nb_leds=0;


    //Informe l'utilisateur que la calibration des capteurs est en cours

     set_led(0,1);
     chThdSleepMilliseconds(200);
     set_rgb_led(LED2,100,0,0);
     chThdSleepMilliseconds(200);
     set_led(1,1);
     chThdSleepMilliseconds(200);
     set_rgb_led(LED4,100,0,0);
     chThdSleepMilliseconds(200);
     set_led(2,1);
     chThdSleepMilliseconds(200);
     set_rgb_led(LED6,100,0,0);
     chThdSleepMilliseconds(200);
     set_led(3,1);
     chThdSleepMilliseconds(200);
     set_rgb_led(LED8,100,0,0);
     chThdSleepMilliseconds(200);

    calibrate_ir();

    uint16_t calibration=0;
    if (get_prox(2) > get_prox(5)) {
    	calibration=get_prox(2);
    } else {
    	calibration=get_prox(5);
    }

    for (int i=0; i<4; ++i) {
         set_led(i,0);
         chThdSleepMilliseconds(200);
         set_rgb_led(i,0,0,0);
         chThdSleepMilliseconds(200);
    }

    while(1){

        time = chVTGetSystemTime();


        if ((get_calibrated_prox(2)/calibration > 0) && (get_calibrated_prox(5)/calibration > 0)) {
        	if (inDoor==0) {
        		++nb_leds;
        	}
    		inDoor=1;

        	set_body_led(1);

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
    }

    //100Hz
    chThdSleepUntilWindowed(time, time + MS2ST(10));
}


void gate_detection_start(void){
	chThdCreateStatic(waGateDetection, sizeof(waGateDetection), NORMALPRIO, GateDetection, NULL);
}
