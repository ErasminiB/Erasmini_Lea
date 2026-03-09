/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "selector.h"
#include "epuck1x/uart/e_uart_char.h"
#include "serial_comm.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // Proximity
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

    //LED
    clear_leds();
    spi_comm_start();

    //set_body_led(2);
    //chThdSleepMilliseconds(1000);

    //Motors
    motors_init();
    get_selector();
    //int blink_count = 0;


    /* Infinite loop. *//*
    while (1)
    {
 	 	int prox0 = get_calibrated_prox(0);
        int prox1 = get_calibrated_prox(1);
        int prox2 = get_calibrated_prox(2);
        int prox5 = get_calibrated_prox(5);
        int prox6 = get_calibrated_prox(6);
        int prox7 = get_calibrated_prox(7);


    	if (get_calibrated_prox(0)>=200 || get_calibrated_prox(7)>=200)
    		{
    			set_front_led(2);
    			left_motor_set_speed(-500);
    			right_motor_set_speed(500);
    		}
    	else if (get_calibrated_prox(1)>=200)
    		{
    			left_motor_set_speed(-500);
    			right_motor_set_speed(500);
    			if (get_calibrated_prox(6)>=200 && get_calibrated_prox(7)>=200)
    			   {
    			    left_motor_set_speed(-500);
    			    right_motor_set_speed(-500);
    			   }
    		}
    	else if (get_calibrated_prox(6)>=200)
    		{
    	    	left_motor_set_speed(500);
    	    	right_motor_set_speed(-500);
    		}

    	else
    	    {
    	    	left_motor_set_speed(500);
    	    	right_motor_set_speed(500);
    	    }
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
/*
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "selector.h"
#include "epuck1x/uart/e_uart_char.h"
#include "serial_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

typedef enum {
    STATE_EXPLORE,
    STATE_BACKWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} robot_state_t;

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

    clear_leds();
    spi_comm_start();

    motors_init();
    get_selector();

    robot_state_t state = STATE_EXPLORE;

    int state_timer = 0;
    int explore_timer = 0;
    int turn_bias = 0;   // alternates left/right to stop repeated loops

    while (1)
    {
        int prox0 = get_calibrated_prox(0);   // front-left
        int prox1 = get_calibrated_prox(1);   // left-front
        int prox2 = get_calibrated_prox(2);   // left
        int prox5 = get_calibrated_prox(5);   // right
        int prox6 = get_calibrated_prox(6);   // right-front
        int prox7 = get_calibrated_prox(7);   // front-right

        int front_blocked = (prox0 > 200 || prox7 > 200);
        int very_blocked  = (prox0 > 250 && prox7 > 250);
        int left_blocked  = (prox6 > 200 || prox5 > 200);
        int right_blocked = (prox1 > 200 || prox2 > 200);

        switch(state)
        {
            case STATE_EXPLORE:
                set_front_led(0);

                // Hard case: likely corner / dead end
                if (very_blocked || (front_blocked && left_blocked && right_blocked))
                {
                    state = STATE_BACKWARD;
                    state_timer = 400;   // ms
                }
                // Front obstacle: choose turn direction
                else if (front_blocked)
                {
                    if (prox0 > prox7) {
                        state = STATE_TURN_RIGHT;
                    } else {
                        state = STATE_TURN_LEFT;
                    }
                    state_timer = 300;
                }
                // Gentle obstacle avoidance
                else if (left_blocked)
                {
                    left_motor_set_speed(450);
                    right_motor_set_speed(150);
                }
                else if (right_blocked)
                {
                    left_motor_set_speed(150);
                    right_motor_set_speed(450);
                }
                else
                {
                    // Exploration mode:
                    // mostly forward, but periodically curve to avoid staying on one path
                    if (explore_timer < 2500) {
                        left_motor_set_speed(500);
                        right_motor_set_speed(420);
                    } else if (explore_timer < 3500) {
                        left_motor_set_speed(420);
                        right_motor_set_speed(500);
                    } else {
                        explore_timer = 0;
                        turn_bias = !turn_bias;
                    }
                }
                break;

            case STATE_BACKWARD:
                set_front_led(2);
                left_motor_set_speed(-350);
                right_motor_set_speed(-350);

                state_timer -= 20;
                if (state_timer <= 0)
                {
                    // after reversing, commit to a turn
                    if (turn_bias) {
                        state = STATE_TURN_LEFT;
                    } else {
                        state = STATE_TURN_RIGHT;
                    }
                    state_timer = 450;
                }
                break;

            case STATE_TURN_LEFT:
                set_front_led(2);
                left_motor_set_speed(-250);
                right_motor_set_speed(350);

                state_timer -= 20;
                if (state_timer <= 0)
                {
                    state = STATE_EXPLORE;
                }
                break;

            case STATE_TURN_RIGHT:
                set_front_led(2);
                left_motor_set_speed(350);
                right_motor_set_speed(-250);

                state_timer -= 20;
                if (state_timer <= 0)
                {
                    state = STATE_EXPLORE;
                }
                break;
        }

        explore_timer += 20;
        chThdSleepMilliseconds(20);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
