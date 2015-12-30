/*
 * Copyright (c) 2015 Christoph Krofitsch, 
 * Practical Robotics Institute Austria
 * 
 * This file is part of HedgehogLightPi.
 * 
 * HedgehogLightPi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * HedgehogLightPi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with HedgehogLightPi. If not, see <http://www.gnu.org/licenses/>.
 */

#include "andrixhwtype3.h"

int analog(uint8_t port) {
	uint8_t send[2];
	send[0] = ANALOG_SENSOR_REQUEST;
	send[1] = port;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 2, &answer, &answerLen);
	int result = ((answer[2] << 8) | answer[3]);
	free(answer);
	return result;
}

void setAnalogPullups(bool p0, bool p1, bool p2, bool p3, bool p4, bool p5, bool p6, bool p7, bool p8, bool p9, bool p10, bool p11, bool p12, bool p13, bool p14, bool p15) {
	uint8_t send[3];
	send[0] = ANALOG_PULLUP_ACTION;
	send[1] = (p15 << 7) | (p14 << 6) | (p13 << 5) | (p12 << 4) | (p11 << 3) | (p10 << 2) | (p9 << 1) | (p8);
	send[2] = (p7 << 7) | (p6 << 6) | (p5 << 5) | (p4 << 4) | (p3 << 3) | (p2 << 2) | (p1 << 1) | (p0);
	userProgramSend(send, 3);
}

bool digital(uint8_t port) {
	uint8_t send[2];
	send[0] = DIGITAL_SENSOR_REQUEST;
	send[1] = port;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 2, &answer, &answerLen);
	bool result = answer[2];
	free(answer);
	return result;
}

void moveAtPower(uint8_t port, int power) {
	uint8_t send[4];
	send[0] = MOTOR_POWER_ACTION;
	send[1] = port;
	send[2] = power > 0 ? 0 : 1;
	send[3] = power > 0 ? (uint8_t) (power*2.55) : (uint8_t) (-power*2.55);
	userProgramSend(send, 4);
}

void brake(uint8_t port, int brakingPower) {
	uint8_t send[3];
	send[0] = MOTOR_BRAKE_ACTION;
	send[1] = port;
	send[2] = (uint8_t) (brakingPower * 2.55);
	userProgramSend(send, 3);
}

void off(uint8_t port) {
	uint8_t send[2];
	send[0] = MOTOR_OFF_ACTION;
	send[1] = port;
	userProgramSend(send, 2);
}

void allOff() {
	uint8_t send[2];
	send[0] = MOTOR_OFF_ACTION;
	uint8_t i;
	for(i=0; i<6; i++) {
		send[1] = i;
		userProgramSend(send, 2);
	}
}

void enableAllServos() {
	uint8_t send[3];
	send[0] = SERVO_ONOFF_ACTION;
	send[2] = 1;
	uint8_t i;
	for(i=0; i<6; i++) {
		send[1] = i;
		userProgramSend(send, 3);
	}
}

void disableAllServos() {
	uint8_t send[3];
	send[0] = SERVO_ONOFF_ACTION;
	send[2] = 0;
	uint8_t i;
	for(i=0; i<6; i++) {
		send[1] = i;
		userProgramSend(send, 3);
	}
}

void enableServo(uint8_t port) {
	uint8_t send[3];
	send[0] = SERVO_ONOFF_ACTION;
	send[1] = port;
	send[2] = 1;
	userProgramSend(send, 3);
}

void disableServo(uint8_t port) {
	uint8_t send[3];
	send[0] = SERVO_ONOFF_ACTION;
	send[1] = port;
	send[2] = 0;
	userProgramSend(send, 3);
}

void setPosition(uint8_t port, int position) {
	uint8_t send[3];
	send[0] = SERVO_DRIVE_ACTION;
	send[1] = port;
	send[2] = (uint8_t) (position*2.55/1.8);
	userProgramSend(send, 3);
}

int controllerBatteryCharge() {
	uint8_t send[1];
	send[0] = CONTROLLER_BATTERY_CHARGE_REQUEST;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 1, &answer, &answerLen);
	int result = (int) (answer[1] / 2.55);
	free(answer);
	return result;
}

bool controllerBatteryChargingState() {
	uint8_t send[1];
	send[0] = CONTROLLER_BATTERY_CHARGING_STATE_REQUEST;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 1, &answer, &answerLen);
	bool result = answer[1];
	free(answer);
	return result;
}

int phoneBatteryCharge() {
	uint8_t send[1];
	send[0] = PHONE_BATTERY_CHARGE_REQUEST;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 1, &answer, &answerLen);
	int result = (int) (answer[1] / 2.55);
	free(answer);
	return result;
}

bool phoneBatteryChargingState() {
	uint8_t send[1];
	send[0] = PHONE_BATTERY_CHARGING_STATE_REQUEST;
	uint8_t *answer;
	uint32_t answerLen;
	userProgramRequest(send, 1, &answer, &answerLen);
	bool result = answer[1];
	free(answer);
	return result;
}
