/*
 * Timers.c
 *
 *  Created on: 24.09.2017
 *      Author: anonymous
 */

#include "globals.h"
#include "defines.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp32-hal-timer.h"

extern "C" {

portMUX_TYPE muxer1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxer2 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timer1ISR() {
	portENTER_CRITICAL_ISR(&muxer1);

	if (dir_M1 != 0) {
		// We generate 1us STEP pulse
		digitalWrite(PIN_MOTOR1_STEP, HIGH);

		if (dir_M1 > 0)
			steps1--;
		else
			steps1++;

		digitalWrite(PIN_MOTOR1_STEP, LOW);
	}

	portEXIT_CRITICAL_ISR(&muxer1);
}
void IRAM_ATTR timer2ISR() {
	portENTER_CRITICAL_ISR(&muxer2);

	if (dir_M2 != 0) {
		// We generate 1us STEP pulse
		digitalWrite(PIN_MOTOR2_STEP, HIGH);

		if (dir_M2 > 0)
			steps2--;
		else
			steps2++;

		digitalWrite(PIN_MOTOR2_STEP, LOW);
	}
	portEXIT_CRITICAL_ISR(&muxer2);
}
}

void initTimers() {

	timer1 = timerBegin(0, 40, true);
	timerAttachInterrupt(timer1, &timer1ISR, true);
	timerAlarmWrite(timer1, ZERO_SPEED, true);

	timer2 = timerBegin(1, 40, true);
	timerAttachInterrupt(timer2, &timer2ISR, true);
	timerAlarmWrite(timer2, ZERO_SPEED, true);

	timerAlarmEnable(timer1);
	timerAlarmEnable(timer2);

}
