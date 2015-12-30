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
 * The functions in this header are directly available to user programs, independent of what hardware
 * controller is used.
 */

#include "axcp.h"

/*
 * Halts the execution for about 'ms' milliseconds.
 * - param ms: number of milliseconds to halt.
 */
void msleep(int ms);

/*
 * Function for user programs to receive 'length' bytes application-specfic custom data into 'buffer'.
 * This is non-blocking, meaning that the user program will only get what has been received already,
 * so customDataAvailable() should be called in order to ensure that enough data is available. If less
 * than 'length' bytes are available, the rest will be filled with zeros.
 */
void readCustomData(uint8_t* buffer, uint32_t length);

/*
 * Function for user programs to check how many application-specific custom data bytes are available,
 * i.e. have been received so far.
 * Return: The number of bytes available.
 */
uint32_t customDataAvailable();

/*
 * Function for user programs to send 'length' bytes application-specific custom data.
 */  
void sendCustomData(uint8_t* buffer, uint32_t length);

