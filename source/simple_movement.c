#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"

#define MAX_VECTORS 10



int main2(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);
    HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);

//    Esc esc1, esc2;
//    EscInit(&esc1, CTIMER2_PERIPHERAL, CTIMER2_PWM_PERIOD_CH, kCTIMER_Match_1);
//    EscInit(&esc2, CTIMER2_PERIPHERAL, CTIMER2_PWM_PERIOD_CH, kCTIMER_Match_3);
//    EscSetSpeed(&esc1, 79.0);
//    EscBrake(&esc1);
//    EscSetSpeed(&esc2, 59.0);
//    EscBrake(&esc2);

//    TestServo();

	int index = 0;
	while(1){
		index ++;
		index %= 4;
		for(volatile int i = 0; i<=8000000;i++){
			(void)i;
		}
		if(index == 0){
			Steer(0);
			HbridgeSpeed(&g_hbridge, 40, 40);
		}
		else if(index == 1){
			Steer(50);
			HbridgeSpeed(&g_hbridge, 40, 40);
		 }
		else if(index == 2){
			Steer(-50);
			HbridgeSpeed(&g_hbridge, 40, 40);
		}
		else if(index == 3){
			Steer(0);
			HbridgeSpeed(&g_hbridge, -40, -40);
		}
	}


}
