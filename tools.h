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

#include <unistd.h>
#include <inttypes.h>

/*
 * Reads exactly 'length' bytes into 'buffer' from 'fd'. Blocks until all bytes have been read.
 * Return: 0 on success or -1 if one read operation returned -1.
 */
int fullRead(int fd, uint8_t* buffer, const int length);

/*
 * Writes exactly 'length' bytes from 'buffer' into 'fd'. Blocks until all bytes have been written.
 * Return: 0 on success or -1 if one write operation returned -1.
 */
int fullWrite(int fd, const uint8_t* buffer, const int length);
