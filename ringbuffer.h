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

#include <inttypes.h>
#include <stdlib.h>

// Struct that holds all information about the ringbuffer
typedef struct {
    int readIndex;
    int writeIndex;
    // The actual buffer
    uint8_t *fifo;
    // size of the buffer
    int size;
} ringbuffer_handler_t;

/*
 * Creates a ringbuffer with length 'size' and returns the handler struct
 */
ringbuffer_handler_t *createFIFO(int size);

/*
 * Calculates the number of free bytes in the ringbuffer handled by 'buffer', i.e. the number of
 * bytes that can be written.
 * Return: the number of free bytes or -1 if 'buffer' is not valid.
 */
int availableFIFO(ringbuffer_handler_t *buffer);

/*
 * Adds a new byte to the ringbuffer handled by 'buffer'.
 * Return: 0 on success or -1 if the ringbuffer is full or 'buffer' is invalid
 */
int appendFIFO(uint8_t data, ringbuffer_handler_t *buffer);

/*
 * Reads a byte from the ringbuffer handled by 'buffer' and deletes it.
 * Return: 0 on success or -1 if the ringbuffer is empty or 'buffer' is invalid.
 */
int readFIFO(uint8_t *data, ringbuffer_handler_t *buffer);

/*
 * Frees all memory taken by the ringbuffer handled by 'buffer'.
 */
void destroyFIFO(ringbuffer_handler_t *buffer);
