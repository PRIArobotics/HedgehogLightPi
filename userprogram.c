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

#include "userprogram.h"

void msleep(int ms) {
	if(ms < 0)
		return;
	usleep(ms * 1000);
}

void readCustomData(uint8_t* data, uint32_t length) {
        uint8_t send[5];
        send[0] = READ_CUSTOM_DATA_REQUEST_SWCINTERN;
        send[1] = ((length >> 24) & 0xFF);
        send[2] = ((length >> 16) & 0xFF);
        send[3] = ((length >> 8) & 0xFF);
        send[4] = (length & 0xFF);
        uint8_t *receive;
        uint32_t receiveLen;
        // Receive the desired data from the parent process holding the ringbuffer
        userProgramRequest(send, 5, &receive, &receiveLen);
        memcpy(data, receive + 1, receiveLen - 1);
        free(receive);
}

uint32_t customDataAvailable() {
        uint8_t send[1];
        send[0] = CUSTOM_DATA_AVAILABLE_REQUEST_SWCINTERN;
        uint8_t* receive;
        uint32_t receiveLen;
        // Recieve the desired information from the parent process holding the ringbuffer.
        userProgramRequest(send, 1, &receive, &receiveLen);
        uint32_t result = ((receive[1] << 24) | (receive[2] << 16) | (receive[3] << 8) | receive[4]);
        free(receive);
        return result;
}

void sendCustomData(uint8_t* buffer, uint32_t length) {
        uint8_t send[length + 1];
        send[0] = SEND_CUSTOM_DATA_ACTION_SWCINTERN;
        memcpy(send + 1, buffer, length);
        userProgramSend(send, length + 1);
}

