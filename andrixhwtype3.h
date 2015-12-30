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
 * This header contains function definitions that are directly available to user programs, i.e. to
 * work with the respective hardware controller these functions should be used.
 */

#include "axcp.h"
#include <stdbool.h>

/*
 * Reads the analog sensor at port number 'port' and returns its value.
 * - param port: port number ranging from 0 to 15.
 * - return: the sensor value ranging from 0 to 1023.
 */
int analog(uint8_t port);

/*
 * Controls the pullup resistors for all 16 analog ports. The boolean parameters represent the resistor
 * setting for each port in incremental order. An inactive pullup resistor means "floating", which is
 * for instance needed for the E.T. distance sensors in the Botball Kit.
 */
void setAnalogPullups(bool p0, bool p1, bool p2, bool p3, bool p4, bool p5, bool p6, bool p7, bool p8, bool p9, bool p10, bool p11, bool p12, bool p13, bool p14, bool p15);

/*
 * Reads the digital sensor at port number 'port' and returns its value. The value is binary and is returned
 * as bool.
 * - param port: port number ranging from 0 to 15.
 * - return: true if the sensor is pressed, false if not.
 */
bool digital(uint8_t port);

/*
 * Controls the output mode for all 16 digital ports. The boolean parameters represent the output modes
 * for each port in incremental order, where true means output and false means input.
 */
void setDigitalOutputMode(bool p0, bool p1, bool p2, bool p3, bool p4, bool p5, bool p6, bool p7, bool p8, bool p9, bool p10, bool p11, bool p12, bool p13, bool p14, bool p15);

/*
 * Sets the specified output 'level' for the digital port at number 'port'.
 * - param port: the port number ranging from 0 to 15.
 * - param level: the desired output level.
 */
void setDigitalOutput(uint8_t port, bool level);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'power'.
 * - param port: the port number ranging from 0 to 5.
 * - param power: desired power ranging from -100 percent (backward) to +100 percent (forward).
 */
void moveAtPower(uint8_t port, int power);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'velocity'.
 * - param port: the port number ranging from 0 to 5.
 * - param velocity: desired velocity ranging from -100 percent (backward) to +100 percent (forward).
 */
void moveAtVelocity(uint8_t port, int velocity);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'power' to the
 * specified absolute target 'position'. After the motor has reached the target position, it stops.
 * - param port: the port number ranging from 0 to 5.
 * - param power: the power ranging from 0 percent to 100 percent.
 * - param position: the absolute target position.
 */
void moveAtPowerToAbsolute(uint8_t port, int power, int position);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'velocity' to the
 * specified absolute target 'position'. After the motor has reached the target position, it stops.
 * - param port: the port number ranging from 0 to 5.
 * - param velocity: the velocity ranging from 0 percent to 100 percent.
 * - param position: the absolute target position.
 */
void moveAtVelocityToAbsolute(uint8_t port, int velocity, int position);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'power', Changing its
 * position by 'deltaPosition'. After the motor has reached its final position, it stops.
 * - param port: the port number ranging from 0 to 5.
 * - param power: the power ranging from 0 percent to 100 percent.
 * - param deltaPosition: the desired position difference.
 */
void moveAtPowerToRelative(uint8_t port, int power, int deltaPosition);

/*
 * Drives the motor connected to the motor port number 'port' at the specified 'velocity', Changing its
 * position by 'deltaPosition'. After the motor has reached its final position, it stops.
 * - param port: the port number ranging from 0 to 5.
 * - param velocity: the velocity ranging from 0 percent to 100 percent.
 * - param deltaPosition: the desired position difference.
 */
void moveAtVelocityToRelative(uint8_t port, int velocity, int deltaPosition);

/*
 * Freezes the motor connected to the motor port number 'port'. The motor then tries to hold its position and
 * applies force if necessary to achieve this.
 * - param port: the port number ranging from 0 to 5.
 */
void freeze(uint8_t port);

/*
 * Brakes the motor connected to the motor port number 'port' with the specified 'brakingPower'.
 * - param port: the port number ranging from 0 to 5.
 * - param brakingPower: the breaking power ranging from 0 percent to 100 percent.
 */
void brake(uint8_t port, int brakingPower);

/*
 * Stops the motor connected to the motor port number 'port'. Stops any movement, freezing or braking, i.e.
 * the motor won't be powered and can move freely.
 * - param port: the port number ranging from 0 to 5.
 */
void off(uint8_t port);

/*
 * Stops all motors connected to the controller. Stops any movement, freezing or braking, i.e. the
 * motors won't be powered and can move freely.
 */
void allOff();

/*
 * Clears the internal position counter for the motor port number 'port', i.e. sets it to zero.
 * - param port: the port number ranging from 0 to 5.
 */
void clearPosition(uint8_t port);

/*
 * Returns the current position of the motor connected to port number 'port'.
 * - param port: the port number ranging from 0 to 5.
 * - return: the current position of the motor.
 */
int getPosition(uint8_t port);

/*
 * Returns the current velocity of the motor connected to port number 'port'.
 * - param port: the port number ranging from 0 to 5.
 * - return: the current velocity of the motor ranging from -100 percent to +100 percent.
 */
int getVelocity(uint8_t port);

/*
 * Activates all servos that are connected to the controller. An activated servo holds its configured
 * position and applies force if necessary.
 */
void enableAllServos();

/*
 * Deactivates all servos that are connected to the controller. A deactivated servo can move freely,
 * independent of what position is set.
 */
void disableAllServos();

/*
 * Activates the servo connected to servo port number 'port'. An activated servo holds its configured
 * position and applies force if necessary.
 * - param port: the port number ranging from 0 to 5.
 */
void enableServo(uint8_t port);

/*
 * Deactivates the servo connected to servo port number 'port'. A deactivated servo can move freely,
 * independent of what position is set.
 * - param port: the port number ranging from 0 to 5.
 */
void disableServo(uint8_t port);

/*
 * Sets the 'position' of the servo connected to servo port number 'port'. If activated, the servo will
 * move to this position and hold it.
 * - param port: the port number ranging from 0 to 5.
 * - param position: the target position ranging from 0 degrees to 180 degrees.
 */
void setPosition(uint8_t port, int position);

/*
 * Returns the current charge of the controller battery.
 * - return: the controller battery charge ranging from 0 percent to 100 percent.
 */
int controllerBatteryCharge();

/*
 * Returns the current charging state of the controller battery.
 * - return: true if the controller battery is charging, false if not.
 */
bool controllerBatteryChargingState();

/*
 * Returns the current charge of the phone battery.
 * - return: the phone battery charge ranging from 0 percent to 100 percent.
 */
int phoneBatteryCharge();

/*
 * Returns the current charging state of the phone battery.
 * - return: true if the phone battery is charging, false if not.
 */
bool phoneBatteryChargingState();
