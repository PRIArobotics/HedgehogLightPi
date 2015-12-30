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

#include "tools.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define PROGRAM_IN_FD 202
#define PROGRAM_OUT_FD 203

// AXCP (and AXDP) opcode definitions
#define NOP 0
#define NOP2 248
#define SEND_CUSTOM_DATA_ACTION_SWCINTERN 5
#define CUSTOM_DATA_AVAILABLE_REQUEST_SWCINTERN 6
#define CUSTOM_DATA_AVAILABLE_REPLY_SWCINTERN 7
#define READ_CUSTOM_DATA_REQUEST_SWCINTERN 8
#define READ_CUSTOM_DATA_REPLY_SWCINTERN 9
#define ANALOG_SENSOR_REQUEST 10
#define ANALOG_SENSOR_REPLY 11
#define ANALOG_SENSOR_SUBSCRIPTION 12
#define ANALOG_SENSOR_UPDATE 13
#define ANALOG_PULLUP_ACTION 14
#define DIGITAL_SENSOR_REQUEST 20
#define DIGITAL_SENSOR_REPLY 21
#define DIGITAL_SENSOR_SUBSCRIPTION 22
#define DIGITAL_SENSOR_UPDATE 23
#define DIGITAL_PULLUP_ACTION 24
#define DIGITAL_OUTPUT_MODE_ACTION 25
#define DIGITAL_OUTPUT_LEVEL_ACTION 26
#define MOTOR_POWER_ACTION 30
#define MOTOR_VELOCITY_ACTION 31
#define MOTOR_POWER_ABSOLUTE_POSITION_ACTION 32
#define MOTOR_VELOCITY_ABSOLUTE_POSITION_ACTION 33
#define MOTOR_POWER_RELATIVE_POSITION_ACTION 34
#define MOTOR_VELOCITY_RELATIVE_POSITION_ACTION 35
#define MOTOR_FREEZE_ACTION 36
#define MOTOR_BRAKE_ACTION 37
#define MOTOR_OFF_ACTION 38
#define MOTOR_POSITION_REQUEST 40
#define MOTOR_POSITION_REPLY 41
#define MOTOR_POSITION_REACHED_ACTION 42
#define MOTOR_POSITION_SUBSCRIPTION 43
#define MOTOR_POSITION_UPDATE 44
#define MOTOR_CLEAR_POSITION_ACTION 45
#define MOTOR_VELOCITY_REQUEST 46
#define MOTOR_VELOCITY_REPLY 47
#define MOTOR_VELOCITY_SUBSCRIPTION 48
#define MOTOR_VELOCITY_UPDATE 49
#define SERVO_ONOFF_ACTION 50
#define SERVO_DRIVE_ACTION 51
#define CONTROLLER_BATTERY_CHARGE_REQUEST 60
#define CONTROLLER_BATTERY_CHARGE_REPLY 61
#define CONTROLLER_BATTERY_CHARGING_STATE_REQUEST 62
#define CONTROLLER_BATTERY_CHARGING_STATE_REPLY 63
#define PHONE_BATTERY_CHARGE_REQUEST 64
#define PHONE_BATTERY_CHARGE_REPLY 65
#define PHONE_BATTERY_CHARGING_STATE_REQUEST 66
#define PHONE_BATTERY_CHARGING_STATE_REPLY 67
#define CONTROLLER_BATTERY_UPDATE 68
#define PHONE_SENSOR_REQUEST 70
#define PHONE_SENSOR_REPLY 71
#define PHONE_SENSOR_AVAILABILITY_REQUEST 72
#define PHONE_SENSOR_AVAILABILITY_REPLY 73
#define PHONE_CAMERA_TAKE_PICTURE_ACTION 80
#define PHONE_CAMERA_GET_BLOB_COUNT_REQUEST 81
#define PHONE_CAMERA_GET_BLOB_COUNT_REPLY 82
#define PHONE_CAMERA_GET_BLOB_REQUEST 83
#define PHONE_CAMERA_GET_BLOB_REPLY 84
#define PHONE_CAMERA_SET_CHANNEL_ACTION 85
#define HW_CONTROLLER_OFF_ACTION 90
#define HW_CONTROLLER_RESET_ACTION 91
#define SW_CONTROLLER_OFF_ACTION 92
#define SW_CONTROLLER_RESET_ACTION 93
#define PHONE_OFF_ACTION 94
#define PHONE_RESET_ACTION 95
#define ERROR_ACTION 96
#define CUSTOM_ACTION 97
#define DEBUG_INFORMATION_UPDATE 100
#define HW_CONTROLLER_TYPE_REQUEST 110
#define HW_CONTROLLER_TYPE_REPLY 111
#define SW_CONTROLLER_TYPE_REQUEST 112
#define SW_CONTROLLER_TYPE_REPLY 113
#define PHONE_TYPE_REQUEST 114
#define PHONE_TYPE_REPLY 115
#define HW_CONTROLLER_SET_MEMORY_ACTION 116
#define ENVIRONMENT_SCAN_SUBSCRIPTION 120
#define ENVIRONMENT_SCAN_HW_CONTROLLER_UPDATE 121
#define ENVIRONMENT_SCAN_SW_CONTROLLER_UPDATE 122
#define ENVIRONMENT_SCAN_PHONE_UPDATE 123
#define CONTROLLER_AUTHENTICATE_REQUEST 124
#define CONTROLLER_AUTHENTICATE_REPLY 125
#define HW_CONTROLLER_GET_MEMORY_REQUEST 126
#define HW_CONTROLLER_GET_MEMORY_REPLY 127
#define PROGRAM_COMPILE_REQUEST 150
#define PROGRAM_COMPILE_REPLY 151
#define PROGRAM_EXECUTE_ACTION 152
#define PROGRAM_COMPILE_EXECUTE_REQUEST 153
#define PROGRAM_COMPILE_EXECUTE_REPLY 154
#define PROGRAMS_FETCH_SUBSCRIPTION 155
#define PROGRAMS_FETCH_UPDATE 156
#define PROGRAMS_FETCH_DONE_UPDATE 157
#define EXECUTION_STARTED_ACTION 160
#define EXECUTION_STOP_ACTION 161
#define EXECUTION_RESTART_ACTION 162
#define EXECUTION_STOPPED_ACTION 163
#define EXECUTION_DONE_ACTION 164
#define EXECUTION_PRINTOUT_ACTION 165
#define EXECUTION_DATA_ACTION 166
#define DEBUGGING_BREAK_ACTION 170
#define DEBUGGING_BREAKED_ACTION 171
#define DEBUGGING_CONTINUE_ACTION 172
#define DEBUGGING_ADD_BREAKPOINT_ACTION 173
#define DEBUGGING_REMOVE_BREAKPOINT_ACTION 174

// AXCP (and AXDP) error code definition
#define ERRORCODE_UNSPECIFIED_OPCODE 1
#define ERRORCODE_ANALOG_PORT_OUT_OF_RANGE 2
#define ERRORCODE_DIGITAL_PORT_OUT_OF_RANGE 3
#define ERRORCODE_MOTOR_PORT_OUT_OF_RANGE 4
#define ERRORCODE_SERVO_PORT_OUT_OF_RANGE 5
#define ERRORCODE_SERVO_IS_OFF 6
#define ERRORCODE_PHONE_SENSOR_TYPE_NOT_SUPPORTED 7
#define ERRORCODE_PHONE_SENSOR_TYPE_DOESNT_EXIST 8
#define ERRORCODE_CHANNEL_NOT_CONFIGURED 9
#define ERRORCODE_NO_BLOB_AT_INDEX 10
#define ERRORCODE_OPERATION_NOT_SUPPORTED 11
#define ERRORCODE_PAYLOAD_LENGTH_OUT_OF_RANGE 12
#define ERRORCODE_INCOMPLETE_COMMAND_TIMEOUT 13
#define ERRORCODE_PROGRAM_NOT_FOUND 150
#define ERRORCODE_A_PROGRAM_IS_ALREADY_RUNNING 151
#define ERRORCODE_NO_HW_CONTROLLER_CONNECTED 152
#define ERRORCODE_PROGRAM_IS_NOT_RUNNING 153
#define ERRORCODE_PROGRAM_IS_NOT_BREAKED 154
#define ERRORCODE_UNSPECIFIED_ERROR 255

/*
 * Returns the payload length of the command with the specified 'opcode'.
 * Return: the payload length, -1 if the command has a variable payload length or -2 if the command is unknown.
 */
int payloadLength(uint8_t opcode);

/*
 * Takes the plain 'command' (opcode + payload) of length 'length', encodes it and sends it through 'fd'.
 * Encoding works according to the AXCP specification, see Excel file.
 * Return: 0 on success, -1 if there was an I/O error and -2 if the command has a fixed payload length which
 * does not correspond to the given length.
 */
int axcpEncodeAndSend(int fd, uint8_t* command, uint32_t length);

/*
 * Receives one full enconded command from 'fd', decodes it and saves the plain command (opcode + payload)
 * in an allocated memory whose address and length will be assigned to 'command' and 'length'. Therefore,
 * both 'command' and 'length' must be valid addreses since they will be dereferenced. In case the
 * received opcode is unknown, the first byte in command is set to this opcode.
 * ATTENTION: The memory for the command will be allocated via malloc(), so don't forget to call
 * free() on the command pointer after you're done!
 * Return: 0 on success, -1 if there was an I/O error or -2 if an unknown opcode was received.
 */
int axcpReceiveAndDecode(int fd, uint8_t **command, uint32_t *length);

/*
 * Must be used by user programs when sending a non-blocking AXCP command,
 * i.e. actions, replies, subscriptions and updates. Does not block and uses axcpEncodeAndSend() function.
 * See axcpEncodeAndSend() as the usage is the same.
 * Return: 0 on success, -1 if there was an I/O error and -2 if the command has a fixed payload length which
 * does not correspond to the given length.
 */
int userProgramSend(uint8_t* command, uint32_t length);

/*
 * Must be used by user programs when sending a blocking AXCP command,
 * i.e. requests. Does block until the reply was received. Uses axcpEncodeAndSend() for sending the request
 * and axcpReceiveAndDecode() for receiving the reply. See axcpEncodeAndSend() for the parameters 'send' and
 * 'sendLen' and axcpReceiveAndDecode() for the parameters 'answer' and 'answerLen' as the usage is the same.
 * Return: 0 on success, -1 if there was an I/O error, -2 if the command has a fixed payload length which
 * does not correspond to the given length or -3 if an unknown opcode was received.
 */
int userProgramRequest(uint8_t* send, uint32_t sendLen, uint8_t** answer, uint32_t* answerLen);
