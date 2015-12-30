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

/* 
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

#include "axcp.h"

int payloadLength(uint8_t opcode) {
	switch(opcode) {
		case NOP: return 0;
		case NOP2: return 0;
		case ANALOG_SENSOR_REQUEST: return 1;
		case ANALOG_SENSOR_REPLY: return 3;
		case ANALOG_SENSOR_SUBSCRIPTION: return -1;
		case ANALOG_SENSOR_UPDATE: return -1;
		case ANALOG_PULLUP_ACTION: return -1;
		case DIGITAL_SENSOR_REQUEST: return 1;
		case DIGITAL_SENSOR_REPLY: return 2;
		case DIGITAL_SENSOR_SUBSCRIPTION: return -1;
		case DIGITAL_SENSOR_UPDATE: return -1;
		case DIGITAL_PULLUP_ACTION: return -1;
		case DIGITAL_OUTPUT_MODE_ACTION: return -1;
		case DIGITAL_OUTPUT_LEVEL_ACTION: return 2;
		case MOTOR_POWER_ACTION: return 3;
		case MOTOR_VELOCITY_ACTION: return 3;
		case MOTOR_POWER_ABSOLUTE_POSITION_ACTION: return 6;
		case MOTOR_VELOCITY_ABSOLUTE_POSITION_ACTION: return 6;
		case MOTOR_POWER_RELATIVE_POSITION_ACTION: return 6;
		case MOTOR_VELOCITY_RELATIVE_POSITION_ACTION: return 6;
		case MOTOR_FREEZE_ACTION: return 1;
		case MOTOR_BRAKE_ACTION: return 2;
		case MOTOR_OFF_ACTION: return 1;
		case MOTOR_POSITION_REQUEST: return 1;
		case MOTOR_POSITION_REPLY: return 5;
		case MOTOR_POSITION_REACHED_ACTION: return 1;
		case MOTOR_POSITION_SUBSCRIPTION: return -1;
		case MOTOR_POSITION_UPDATE: return -1;
		case MOTOR_CLEAR_POSITION_ACTION: return 1;
		case MOTOR_VELOCITY_REQUEST: return 1;
		case MOTOR_VELOCITY_REPLY: return 3;
		case MOTOR_VELOCITY_SUBSCRIPTION: return -1;
		case MOTOR_VELOCITY_UPDATE: return -1;
		case SERVO_ONOFF_ACTION: return 2;
		case SERVO_DRIVE_ACTION: return 2;
		case CONTROLLER_BATTERY_CHARGE_REQUEST: return 0;
		case CONTROLLER_BATTERY_CHARGE_REPLY: return 1;
		case CONTROLLER_BATTERY_CHARGING_STATE_REQUEST: return 0;
		case CONTROLLER_BATTERY_CHARGING_STATE_REPLY: return 1;
		case PHONE_BATTERY_CHARGE_REQUEST: return 0;
		case PHONE_BATTERY_CHARGE_REPLY: return 1;
		case PHONE_BATTERY_CHARGING_STATE_REQUEST: return 0;
		case PHONE_BATTERY_CHARGING_STATE_REPLY: return 1;
		case CONTROLLER_BATTERY_UPDATE: return 2;
		case PHONE_SENSOR_REQUEST: return 1;
		case PHONE_SENSOR_REPLY: return -1;
		case PHONE_SENSOR_AVAILABILITY_REQUEST: return 0;
		case PHONE_SENSOR_AVAILABILITY_REPLY: return 4;
		case PHONE_CAMERA_TAKE_PICTURE_ACTION: return 0;
		case PHONE_CAMERA_GET_BLOB_COUNT_REQUEST: return 1;
		case PHONE_CAMERA_GET_BLOB_COUNT_REPLY: return 2;
		case PHONE_CAMERA_GET_BLOB_REQUEST: return 2;
		case PHONE_CAMERA_GET_BLOB_REPLY: return 10;
		case PHONE_CAMERA_SET_CHANNEL_ACTION: return 7;
		case HW_CONTROLLER_OFF_ACTION: return 0;
		case HW_CONTROLLER_RESET_ACTION: return 0;
		case SW_CONTROLLER_OFF_ACTION: return 0;
		case SW_CONTROLLER_RESET_ACTION: return 0;
		case PHONE_OFF_ACTION: return 0;
		case PHONE_RESET_ACTION: return 0;
		case ERROR_ACTION: return 2;
		case CUSTOM_ACTION: return -1;
		case DEBUG_INFORMATION_UPDATE: return -1;
		case HW_CONTROLLER_TYPE_REQUEST: return 0;
		case HW_CONTROLLER_TYPE_REPLY: return 1;
		case SW_CONTROLLER_TYPE_REQUEST: return 0;
		case SW_CONTROLLER_TYPE_REPLY: return 1;
		case PHONE_TYPE_REQUEST: return 0;
		case PHONE_TYPE_REPLY: return 1;
		case HW_CONTROLLER_SET_MEMORY_ACTION: return -1;
		case ENVIRONMENT_SCAN_SUBSCRIPTION: return 0;
		case ENVIRONMENT_SCAN_HW_CONTROLLER_UPDATE: return 33;
		case ENVIRONMENT_SCAN_SW_CONTROLLER_UPDATE: return 1;
		case ENVIRONMENT_SCAN_PHONE_UPDATE: return 1;
		case CONTROLLER_AUTHENTICATE_REQUEST: return -1;
		case CONTROLLER_AUTHENTICATE_REPLY: return 1;
		case HW_CONTROLLER_GET_MEMORY_REQUEST: return 1;
		case HW_CONTROLLER_GET_MEMORY_REPLY: return -1;
		case PROGRAM_COMPILE_REQUEST: return -1;
		case PROGRAM_COMPILE_REPLY: return -1;
		case PROGRAM_EXECUTE_ACTION: return 34;
		case PROGRAM_COMPILE_EXECUTE_REQUEST: return -1;
		case PROGRAM_COMPILE_EXECUTE_REPLY: return -1;
		case PROGRAMS_FETCH_SUBSCRIPTION: return 0;
		case PROGRAMS_FETCH_UPDATE: return -1;
		case PROGRAMS_FETCH_DONE_UPDATE: return 0;
		case EXECUTION_STARTED_ACTION: return 34;
		case EXECUTION_STOP_ACTION: return 34;
		case EXECUTION_RESTART_ACTION: return 34;
		case EXECUTION_STOPPED_ACTION: return 34;
		case EXECUTION_DONE_ACTION: return 38;
		case EXECUTION_PRINTOUT_ACTION: return -1;
		case EXECUTION_DATA_ACTION: return -1;
		case DEBUGGING_BREAK_ACTION: return 34;
		case DEBUGGING_BREAKED_ACTION: return -1;
		case DEBUGGING_CONTINUE_ACTION: return 34;
		case DEBUGGING_ADD_BREAKPOINT_ACTION: return 36;
		case DEBUGGING_REMOVE_BREAKPOINT_ACTION: return 36;

		case CUSTOM_DATA_AVAILABLE_REQUEST_SWCINTERN: return 0;
		case CUSTOM_DATA_AVAILABLE_REPLY_SWCINTERN: return 4;
		case READ_CUSTOM_DATA_REQUEST_SWCINTERN: return 4;
		case READ_CUSTOM_DATA_REPLY_SWCINTERN: return -1;
    case SEND_CUSTOM_DATA_ACTION_SWCINTERN: return -1;
		default: return -2;
	}
}

int axcpEncodeAndSend(int fd, uint8_t* command, uint32_t length) {

	// If command has variable payload length
	if(payloadLength(command[0]) == -1) {
                uint8_t pl[1];

		// write opcode
                if(fullWrite(fd, command, 1) == -1)
                        return -1;

		// loop that is executed as long as full 255-byte payloads are to be sent
                int curIndex = 1;
                while(length - curIndex >= 255) {
                        pl[0] = 255;
                        if(fullWrite(fd, pl, 1) == -1)
                                return -1;
                        if(fullWrite(fd, command+curIndex, pl[0]) == -1)
                                return -1;
                        curIndex += pl[0];
                }

		// Send remaining payload which is smaller than 255 bytes or even zero
                pl[0] = length - curIndex;
                if(fullWrite(fd, pl, 1) == -1)
                        return -1;
                if(fullWrite(fd, command+curIndex, pl[0]) == -1)
                        return -1;

	// Send full command at once
        } else if (payloadLength(command[0]) > -1) {
		// Check if specified length equals command length definitions
		if((uint32_t) payloadLength(command[0]) != length - 1)
			return -2;
                if(fullWrite(fd, command, length) == -1)
			return -1;
	}

	//uint32_t i;
	//for(i=0; i < length; i++)
  	//	printf("%d, ", command[i]);
	//printf("written to %d!\n", fd);     // <----

	return 0;
}

int axcpReceiveAndDecode(int fd, uint8_t **command, uint32_t *length) {

	uint8_t *buffer = (uint8_t*) malloc(256);

	// read opcode
        if(fullRead(fd, buffer, 1) == -1)
        	return -1;

        int pl = payloadLength(buffer[0]);
	// unknown opcode
	if(pl == -2) {
		*command = buffer;
		*length = 1;
		return -2;
	}

	// If command has variable payload length
        if(pl == -1) {
		// Read first payload length byte
		uint8_t pl_rx[1];
		int curIndex = 1;
		// Loop that is executed as long as 255 byte payload pieces come in
		do {
	       		if(fullRead(fd, pl_rx, 1) == -1)
				return -1;
			buffer = (uint8_t*) realloc(buffer, curIndex + pl_rx[0]);
	                if(fullRead(fd, buffer+curIndex, pl_rx[0]) == -1)
				return -1;
			curIndex += pl_rx[0];
		} while(pl_rx[0] == 255);
		*length = curIndex;
	// Read entire command
	} else if(pl > -1) {
        	if(fullRead(fd, buffer+1, pl) == -1)
			return -1;
		*length = pl + 1;
	}
	*command = buffer;

	//uint32_t i;
	//for(i=0; i < *length; i++)
    //	printf("%d, ", buffer[i]);
	//printf("read %d bytes from %d!\n", *length, fd);     // <----

	return 0;
}

int userProgramSend(uint8_t* command, uint32_t length) {
	return axcpEncodeAndSend(PROGRAM_OUT_FD, command, length);
}

int userProgramRequest(uint8_t* send, uint32_t sendLen, uint8_t** answer, uint32_t* answerLen) {
	int result = axcpEncodeAndSend(PROGRAM_OUT_FD, send, sendLen);
	if(result < 0)
		return result;
	// It is assumed that the pipe connection between user programs and andrixswc answers
        // directly to requests, i.e. no other commands can be transmitted between request and reply.
	// This assumption is guaranteed by the user progams via only calling this function when requesting
        // and by andrixswc via ensuring that no commands can intervene.
	result = axcpReceiveAndDecode(PROGRAM_IN_FD, answer, answerLen);
	if(result == -2)
		return -3;
	return result;
}

