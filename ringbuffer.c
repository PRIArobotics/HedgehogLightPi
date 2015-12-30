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

#include "ringbuffer.h"

ringbuffer_handler_t *createFIFO(int size){
    ringbuffer_handler_t *buffer = (ringbuffer_handler_t *)malloc(sizeof(ringbuffer_handler_t));
    buffer->readIndex=0;
    buffer->writeIndex=0;
    buffer->fifo = (uint8_t*) malloc(sizeof(uint8_t) * size);
    buffer->size = size;
    return buffer;
}

void destroyFIFO(ringbuffer_handler_t *buffer) {
    if(buffer) {
        free(buffer->fifo);
        free(buffer);
    }
}

int availableFIFO(ringbuffer_handler_t *buffer) {
    // checks validity of ringbuffer
    if(!buffer)
        return -1;
    if(buffer->writeIndex >= buffer->readIndex)
	return buffer->writeIndex - buffer->readIndex;
    else
        return buffer->size - (buffer->readIndex - buffer->writeIndex);
}

int appendFIFO(uint8_t data, ringbuffer_handler_t *buffer) {
    // checks validity of ringbuffer
    if(!buffer)
	return -1;
    // A write must not cause readIndex == writeIndex. If so, the ringbuffer is full.
    if(buffer->readIndex == buffer->writeIndex + 1 || (buffer->readIndex == 0 && buffer->writeIndex == buffer->size - 1))
        return -1;
    buffer->fifo[buffer->writeIndex] = data;
    // Advance writeIndex
    buffer->writeIndex++;
    if(buffer->writeIndex==buffer->size)
        buffer->writeIndex = 0;
    return 0;
}

int readFIFO(uint8_t *data, ringbuffer_handler_t *buffer) {
    // checks validity of ringbuffer
    if(!buffer)
	return -1;
    // If readIndex == writeIndex, the ringbuffer is empty.
    if(buffer->readIndex == buffer->writeIndex)
	return -1;
    *data = buffer->fifo[buffer->readIndex];
    // Advance readIndex
    buffer->readIndex++;
    if(buffer->readIndex == buffer->size)
	buffer->readIndex = 0;
    return 0;
}

