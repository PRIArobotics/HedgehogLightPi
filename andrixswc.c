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
 * This program implements the software controller in the Hedgehog architecture for a Raspberry PI.
 * Communication with the HWC or HLC is done via UART and user programs will be executed as simple childs.
 * The main control flow is a consumer pattern (via polling) for incoming commands, either from UART, childs
 * or CLI, appropriate handling of them and returning to polling. All single-threaded.
 * Even though AXCP specification recommends command requests to be blocking until to according reply
 * was received, requests to HWC or HLC here are not blocking and will be noted until a reply was received.
 */

#include "andrixswc.h"

int debugger_pid;
int debugger_wfd;
int debugger_attached = 0;
int debugger_breaked = 0;

int uprog_cmd_wfd = -1;
int program_pid = -1;
uint16_t currVersion;
char currName[32];
int restart = 0;

ringbuffer_handler_t* customDataBuffer;

int replyOpcode = -1;
int replyPort = -1;
uint8_t hwctype = 0;

struct pollfd pfds[5];

void bailOut(char* message, ...) {
	va_list ap;
	if(pfds[0].fd != -1)
		close(pfds[0].fd);
	if(pfds[1].fd != -1)
		close(pfds[1].fd);
	if(uprog_cmd_wfd != -1)
		close(uprog_cmd_wfd);
	if(pfds[2].fd != -1)
		close(pfds[2].fd);
	va_start(ap, message);
	vfprintf(stdout, message, ap);
	va_end(ap);
	printf("%d ", errno);
	printf(strerror(errno));
	printf("\n");
	exit(EXIT_FAILURE);
}

void writeUART(uint8_t* command, uint32_t length) {
	int result = axcpEncodeAndSend(pfds[0].fd, command, length);
  printf("Write to UART opcode %d: ", command[0]);
  uint32_t i;
  for(i=1; i<length; i++)
    printf("%d,", command[i]);
  printf("\n");
  
	if(result == -1)
		bailOut("UART write failed\n");
	if(result == -2)
		bailOut("UART write: specified length doesn't equal command length specification\n");
}

/*
 * - return: 0 on success, 1 on compilation failure, -1 on precondition failure
 */
int compileProgram(const char *name, const uint16_t version, const char *code, const uint32_t codeLength, uint8_t** msg, uint32_t* msgLength) {

	// Check if a hardware controller is connected that can be compiled against
	if(hwctype == 0)
		return -1;

	// Retrieve program name and program version
	char localName[33];
	memcpy(localName, name, 32);
	localName[32] = '\0';
	int i;
	for(i = 31; localName[i] == ' '; i--)
		localName[i] = '\0';

	// Create folder for program if not present
	char path[128];
	snprintf(path, 128, "./%s/", localName);
	mkdir(path, 0777);

	// Open source code file for writing
	snprintf(path, 128, "./%s/%s_v%d.c", localName, localName, version);
	int source_fd = open(path, O_CREAT | O_WRONLY | O_TRUNC, S_IRWXU | S_IRGRP | S_IROTH);
	if(source_fd == -1)
		bailOut("Unable to create source file\n");

	// First, write additional include statements into source file
        // A general user program library and one for the connected HWC will be included.
	char include[64];
	int inclLen = sprintf(include, "#include \"../andrixhwtype%d.h\"\n", hwctype);
	printf("\n");
	printf("Inserting code: %s", include);// <---
	if(fullWrite(source_fd, (uint8_t*)include, inclLen) == -1)
		bailOut("Unable to write source code\n");
	inclLen = sprintf(include, "#include \"../userprogram.h\"\n\n");
	printf("Inserting code: %s", include);// <---
	if(fullWrite(source_fd, (uint8_t*)include, inclLen) == -1)
		bailOut("Unable to write source code\n");

	// Write the actual source code into source file
	if(fullWrite(source_fd, (uint8_t*) code, codeLength) == -1)
		bailOut("Unable to write source code\n");
	close(source_fd);
	printf("Saving %s\n",path); // <---

	// Open file for gcc output, both for compiling and linking
	snprintf(path, 128, "./%s/compiler_output", localName);
	int gcc_file = open(path, O_CREAT | O_WRONLY | O_TRUNC, S_IRWXU | S_IRGRP | S_IROTH);
	if(gcc_file == -1)
		bailOut("Unable to create file\n");

	// useful file names
	char sourcefile[128];
	char objectfile[128];
	char binaryfile[128];
	char hwctypefile[128];
	snprintf(sourcefile, 128, "./%s/%s_v%d.c", localName, localName, version);
	snprintf(objectfile, 128, "./%s/%s_v%d.o", localName, localName, version);
	snprintf(binaryfile, 128, "./%s/%s_v%d", localName, localName, version);
	snprintf(hwctypefile, 128, "./andrixhwtype%d.o", hwctype);

	printf("Building...\n");        // <-----
	printf("gcc -Wall -ggdb3 -std=c99 -pedantic -c -o %s %s\n", objectfile, sourcefile);   // <-----

	// Execute the compiler in a separate process.
	int pid = fork();
	if(pid < 0) {
		bailOut("Failed to fork compile\n");
	} else if(pid == 0) {
		// Redirect STDERR of compiler to have all warnings and errors in the right file and start compiling
		dup2(gcc_file, STDERR_FILENO);
		close(gcc_file);
		execlp("gcc", "gcc", "-Wall", "-ggdb3", "-std=c99", "-pedantic", "-c", "-o", objectfile, sourcefile, NULL);
		bailOut("Child exec fail\n");
	}

	// Wait for the result of the compiler.
	int compilerStatus;
	int linkerStatus;
	if(waitpid(pid, &compilerStatus, 0) < 0)
		bailOut("Couldn't wait for child\n");
	printf("Program compiled with status %d\n", compilerStatus); // <---

	printf("gcc -o %s %s %s %s %s %s\n", binaryfile, objectfile, "./tools.o", "./axcp.o", "./userprogram.o", hwctypefile);

	// If compilation successful, execute the linker in a separate process
	if(compilerStatus == 0) {
		pid = fork();
		if(pid < 0) {
			bailOut("Failed to fork linker\n");
		} else if(pid == 0) {
			// Set linker STDERR output at the end of the gcc file and start linking.
			lseek(gcc_file, 0, SEEK_END);
			dup2(gcc_file, STDERR_FILENO);
			close(gcc_file);
			execlp("gcc", "gcc", "-o", binaryfile, objectfile, "./tools.o", "./axcp.o", "./userprogram.o", hwctypefile, NULL);
			bailOut("Child exec fail\n");
		}

		// Wait for the results of the linker
		if(waitpid(pid, &linkerStatus, 0) < 0)
			bailOut("Couldn't wait for child\n");
		printf("Program linked with status %d\n", linkerStatus); // <---
	}

	// Find out length of gcc file
	off_t len = lseek(gcc_file, 0, SEEK_END);
	if(len < 0)
		bailOut("Couldn't read length of compiler file\n");
	*msgLength = (uint32_t) len;
	close(gcc_file);

	// Open gcc file for reading and read allocated buffer
	gcc_file = open(path, O_RDONLY);
	if(gcc_file == -1)
		bailOut("Unable to open compiler file\n");
	*msg = (uint8_t*) malloc(len);
	if(fullRead(gcc_file, *msg, len) == -1)
		bailOut("Unable to read compiler file\n");
	close(gcc_file);

	return (compilerStatus == 0 && linkerStatus == 0) ? 0 : 1;
}

/*
 * Length of parameter name is implicitly assumed to be 32.
 * - return: 0 on success, -1 if a program is already running, -2 if the program wasn't found 
 */
int executeProgram(const char *name, const uint16_t version) {

	if(pfds[1].fd != -1 || pfds[2].fd != -1)
		return -1;

	// Retrieve program name and program version
	char localName[33];
	memcpy(localName, name, 32);
	localName[32] = '\0';
	int i;
	for(i = 31; localName[i] == ' '; i--)
		localName[i] = '\0';

	// Check if the program is there
	char path[128];
	snprintf(path, 128, "./%s/%s_v%d", localName, localName, version);
	int test_fd = open(path, O_RDONLY);
	if(test_fd == -1)
		return -2;
	close(test_fd);

	// Open communication pipes to the program
	int rpipe[2];
	int wpipe[2];
	int outpipe[2];
	if(pipe(rpipe) < 0)
		bailOut("Failed to open read pipe\n");
	if(pipe(wpipe) < 0)
		bailOut("Failed to open write pipe\n");
	if(pipe(outpipe) < 0)
		bailOut("Failed to open out pipe\n");

	// Start child process
	int pid = fork();
	if(pid < 0) {
		bailOut("Failed to fork\n");
	} else if(pid == 0) {
		// Redirect child fds and start user program
		close(rpipe[0]);
		close(wpipe[1]);
		close(outpipe[0]);
		if(dup2(rpipe[1], PROGRAM_OUT_FD) == -1)
			bailOut("Child write dup2 failed\n");
		if(dup2(wpipe[0], PROGRAM_IN_FD) == -1)
			bailOut("Child read dup2 failed\n");
		if(dup2(outpipe[1], STDOUT_FILENO) == -1)
			bailOut("Child stdout dup2 failed\n");
		if(dup2(outpipe[1], STDERR_FILENO) == -1)
			bailOut("Child stderr dup2 failed\n");
		close(rpipe[1]);
		close(wpipe[0]);
		close(outpipe[1]);
		execlp("stdbuf", "stdbuf", "-o0", "-e0", path, NULL);
		bailOut("Child exec fail\n");
	}

	// init communication fds and global variables
	close(rpipe[1]);
	close(wpipe[0]);
	close(outpipe[1]);
	uprog_cmd_wfd = wpipe[1];
	pfds[1].fd = rpipe[0];
	pfds[2].fd = outpipe[0];
	program_pid = pid;
	currVersion = version;
	memcpy(currName, name, 32);
	customDataBuffer = createFIFO(CUSTOM_DATA_BUFFER_SIZE);
	printf("Program %s successfully started with pid %d\n", localName, program_pid); // <---

  // Prepare debugger for executable
  char* gdbsend = "delete\n";           // delete all previously set breakpoints
	if(fullWrite(debugger_wfd, (uint8_t*) gdbsend, strlen(gdbsend)) == -1)
    bailOut("Failed to command debugger.");	
	char dbg_cmd[134];
	int len = snprintf(dbg_cmd, 134, "file %s\n", path);
	if(fullWrite(debugger_wfd, (uint8_t*) dbg_cmd, len) == -1)
		bailOut("Failed to command debugger.");
	debugger_attached = 0;
	debugger_breaked = 0;

	return 0;
}

int uart_cmd_received(uint8_t* command, uint32_t length) {
	switch(command[0]) {
	case ANALOG_SENSOR_REPLY:
	case DIGITAL_SENSOR_REPLY:
	case MOTOR_POSITION_REPLY:
	case MOTOR_VELOCITY_REPLY:
		if(replyOpcode == command[0] && replyPort == command[1]) {
			replyOpcode = -1;
			replyPort = -1;
      if(uprog_cmd_wfd != -1) {
        int result = axcpEncodeAndSend(uprog_cmd_wfd, command, length);
			  if(result == -1)
			 	  bailOut("I/O error when forwarding to pipe\n");
			  if(result == -2)
				  bailOut("Payload length inconsistency when forwarding to pipe\n");
      }
		}
		break;
	case CONTROLLER_BATTERY_CHARGE_REPLY:
	case CONTROLLER_BATTERY_CHARGING_STATE_REPLY:
	case PHONE_BATTERY_CHARGE_REPLY:
	case PHONE_BATTERY_CHARGING_STATE_REPLY:
		if(replyOpcode == command[0]) {
      replyOpcode = -1;
      if(uprog_cmd_wfd != -1) {
			  int result = axcpEncodeAndSend(uprog_cmd_wfd, command, length);
			  if(result == -1)
				  bailOut("I/O error when forwarding to pipe\n");
			  if(result == -2)
				  bailOut("Payload length inconsistency when forwarding to pipe\n");
			}
		}
		break;
	case ANALOG_SENSOR_UPDATE:
		printf("ANALOG SENSOR UPDATE\n");
		break;
	case DIGITAL_SENSOR_UPDATE:
		printf("DIGITAL SENSOR UPDATE\n");
		break;
	case MOTOR_POSITION_UPDATE:
		printf("MOTOR POSITION UPDATE\n");
		break;
	case MOTOR_VELOCITY_UPDATE:
		printf("MOTOR VELOCITY UPDATE\n");
		break;
	case SW_CONTROLLER_RESET_ACTION:
		system("ls -d ./*/ | xargs rm -r");
		hwctype = 0;
		break;
	case SW_CONTROLLER_OFF_ACTION:
		free(command);
		return 1;
	case ERROR_ACTION:
		printf("ERROR ACTION\n");
		printf("Error code: %d\n", command[1]);
		printf("Causing opcode: %d\n", command[2]);
		break;
	case HW_CONTROLLER_TYPE_REPLY:
		hwctype = command[1];
		break;
	case SW_CONTROLLER_TYPE_REQUEST: {
		uint8_t answer[2];
		answer[0] = SW_CONTROLLER_TYPE_REPLY;
		answer[1] = 1;
		writeUART(answer, 2);
		break;
	} case PROGRAM_COMPILE_REQUEST: {
		printf("PROGRAM COMPILE REQUEST\n"); // <---

		// Compose answer and send to HLC
		uint8_t* msg;
		uint32_t msgLength;
		uint16_t version = (command[33] << 8) | command[34];
		int result = compileProgram((char*) (command + 1), version, (char*) (command + 35), length - 35, &msg, &msgLength);

		// if there's no hardware controller to compile against
		if(result == -1) {
			// Request a hardware controller sign.
			uint8_t send1[1];
			send1[0] = HW_CONTROLLER_TYPE_REQUEST;
			writeUART(send1, 1);
			// Send to HLC that operation failed.
			uint8_t send2[3];
			send2[0] = ERROR_ACTION;
			send2[1] = ERRORCODE_NO_HW_CONTROLLER_CONNECTED;
			send2[2] = PROGRAM_COMPILE_REQUEST;
			writeUART(send2, 3);
			break;
		}

		uint8_t answer[msgLength + 36];
		answer[0] = PROGRAM_COMPILE_REPLY;
		memcpy(answer + 1, command + 1, 32);
		memcpy(answer + 33, command + 33, 2);
		answer[35] = (uint8_t) result;
		memcpy(answer + 36, msg, msgLength);
		free(msg);
		writeUART(answer, msgLength + 36);

		break;
	} case PROGRAM_EXECUTE_ACTION: {

		printf("PROGRAM EXECUTE ACTION\n"); // <---

		uint16_t version = (command[33] << 8) | command[34];
		int result = executeProgram((char*) (command + 1), version);
		// If a program is already running
		if(result == -1) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_A_PROGRAM_IS_ALREADY_RUNNING;
			send[2] = PROGRAM_EXECUTE_ACTION;
			writeUART(send, 3);
			break;
		}
		// If the program wasn't found
		if(result == -2) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_NOT_FOUND;
			send[2] = PROGRAM_EXECUTE_ACTION;
			writeUART(send, 3);
			break;
		}

		uint8_t send[35];
		send[0] = EXECUTION_STARTED_ACTION;
		memcpy(send + 1, currName, 32);
		send[33] = (currVersion >> 8) & 0xFF;
		send[34] = currVersion & 0xFF;
		writeUART(send, 35);
		break;
	} case PROGRAM_COMPILE_EXECUTE_REQUEST: {

		printf("PROGRAM COMPILE EXECUTE REQUEST\n"); // <---

		uint8_t* msg;
		uint32_t msgLength;
		uint16_t version = (command[33] << 8) | command[34];
		int result = compileProgram((char*) (command + 1), version, (char*) (command + 35), length - 35, &msg, &msgLength);

		// if there's no hardware controller to compile against
		if(result == -1) {
			// Request a hardware controller sign.
			uint8_t send1[1];
			send1[0] = HW_CONTROLLER_TYPE_REQUEST;
			writeUART(send1, 1);
			// Send to HLC that operation failed.
			uint8_t send2[3];
			send2[0] = ERROR_ACTION;
			send2[1] = ERRORCODE_NO_HW_CONTROLLER_CONNECTED;
			send2[2] = PROGRAM_COMPILE_EXECUTE_REQUEST;
			writeUART(send2, 3);
			break;
		}

		// Compose compilation answer and send to HLC
		uint8_t answer[msgLength + 36];
		answer[0] = PROGRAM_COMPILE_EXECUTE_REPLY;
		memcpy(answer + 1, command + 1, 32);
		memcpy(answer + 33, command + 33, 2);
		answer[35] = (uint8_t) result;
		memcpy(answer + 36, msg, msgLength);
		free(msg);
		writeUART(answer, msgLength + 36);

		// If compilation was unsuccessfuly, stop
		if(answer[35] != 0)
			break;

		result = executeProgram((char*) (command + 1), version);
		// If a program is already running
		if(result == -1) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_A_PROGRAM_IS_ALREADY_RUNNING;
			send[2] = PROGRAM_COMPILE_EXECUTE_REQUEST;
			writeUART(send, 3);
			break;
		}
		// If the program wasn't found
		if(result == -2) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_NOT_FOUND;
			send[2] = PROGRAM_COMPILE_EXECUTE_REQUEST;
			writeUART(send, 3);
			break;
		}

		uint8_t send[35];
		send[0] = EXECUTION_STARTED_ACTION;
		memcpy(send + 1, currName, 32);
		send[33] = (currVersion >> 8) & 0xFF;
		send[34] = currVersion & 0xFF;
		writeUART(send, 35);
		break;
	} case PROGRAMS_FETCH_SUBSCRIPTION: {
		printf("PROGRAMS FETCH SUBSCRIPTION\n"); // <---

		DIR* root_dir = opendir(".");
		struct dirent* root;
		while((root = readdir(root_dir)) != NULL) {
			if(root -> d_type != DT_DIR)
				continue;
			if(strncmp(root -> d_name, ".", 1) == 0 || strncmp(root -> d_name, "..", 2) == 0)
				continue;
			char dirname[64];
			snprintf(dirname, 64, "./%s", root -> d_name);
			DIR* prog_dir = opendir(dirname);
			struct dirent* prog;
			while((prog = readdir(prog_dir)) != NULL) {
				int len = strlen(prog -> d_name);
				if((prog -> d_name)[len-2] != '.' || (prog -> d_name)[len-1] != 'c') 
					continue;

				int i;
				for(i = len-1; (prog -> d_name)[i] != 'v'; i--);
				uint16_t version = (uint16_t) atoi((prog -> d_name) + i + 1);

				char path[128];
				snprintf(path, 128, "./%s/%s", root -> d_name, prog -> d_name);
				int temp_fd = open(path, O_RDONLY);
				if(temp_fd == -1)
					bailOut("Failed to open source file\n");

				int startOffset = 0, numberOfLines = 0;
				char tmpChar[1];
				while(1) {
					if(fullRead(temp_fd, (uint8_t*) tmpChar, 1) == -1)
						bailOut("Failed to read source file\n");
					startOffset++;
					if(tmpChar[0] == '\n')
						numberOfLines++;
					if(numberOfLines == 3)
						break;
				}
				int filelen = lseek(temp_fd, 0, SEEK_END) - startOffset;
				lseek(temp_fd, startOffset, SEEK_SET);

				uint8_t send[filelen + 35];
				send[0] = PROGRAMS_FETCH_UPDATE;
				len = strlen(root -> d_name);
				memcpy((char*)(send + 1), root -> d_name, len);
				for(i = 1 + len; i < 33; i++)
					send[i] = ' ';
				send[33] = (version >> 8) & 0xFF;
				send[34] = version & 0xFF;

				if(fullRead(temp_fd, send+35, filelen) == -1)
					bailOut("Failed to read source file\n");
				writeUART(send, filelen + 35);
			}
		}

		uint8_t send[1];
		send[0] = PROGRAMS_FETCH_DONE_UPDATE;
		writeUART(send, 1);

		break;
	} case EXECUTION_STOP_ACTION: {
		printf("EXECUTION STOP ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = EXECUTION_STOP_ACTION;
			writeUART(send, 3);
			break;
		}

		if (!debugger_breaked) {
			// Signal child process; SIGTERM also automatically detaches the debugger
			kill(program_pid, SIGTERM);
		} else {
			// Tell debugger to terminate child
			char* gdbsend = "signal SIGTERM\n";
			fullWrite(debugger_wfd, (uint8_t*) gdbsend, strlen(gdbsend));
		}

		break;
	} case EXECUTION_RESTART_ACTION: {
		printf("EXECUTION RESTART ACTION\n"); // <---

		if(program_pid < 0) {
			int result = executeProgram(currName, currVersion);
			if(result == -2) {
				uint8_t send[3];
				send[0] = ERROR_ACTION;
				send[1] = ERRORCODE_PROGRAM_NOT_FOUND;
				send[2] = EXECUTION_RESTART_ACTION;
				writeUART(send, 3);
			} else if(result == 0) {
				uint8_t send[35];
				send[0] = EXECUTION_STARTED_ACTION;
				memcpy(send + 1, currName, 32);
				send[33] = (currVersion >> 8) & 0xFF;
				send[34] = currVersion & 0xFF;
				writeUART(send, 35);
			} else {
				bailOut("Impossible return value from executeProgram\n");
			}
		} else {
			// Signal child process; SIGTERM also automatically detaches the debugger
			kill(program_pid, SIGTERM);
			restart = 1;
		}
		break;
	} case EXECUTION_DATA_ACTION: {
		printf("EXECUTION DATA ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = EXECUTION_DATA_ACTION;
			writeUART(send, 3);
			break;
		}
		int customDataLength = length - 35;
		int i;
		for(i=0; i<customDataLength; i++) {
			if(appendFIFO(command[35+i], customDataBuffer) == -1) {
				// TODO buffer full: send error action
				break;
			}
		}
		break;
	} case DEBUGGING_BREAK_ACTION: {
		printf("DEBUGGING BREAK ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = DEBUGGING_BREAK_ACTION;
			writeUART(send, 3);
			break;
		}

		if(!debugger_attached) {
			char dbg_cmd[20];
			int len = snprintf(dbg_cmd, 20, "attach %d\n", program_pid);
			if(fullWrite(debugger_wfd, (uint8_t*) dbg_cmd, len) == -1)
				bailOut("Failed to command debugger.");
			debugger_attached = 1;
			debugger_breaked = 1;
		} else {
			kill(program_pid, SIGINT);
			debugger_breaked = 1;
		}

		break;
	} case DEBUGGING_CONTINUE_ACTION: {
		printf("DEBUGGING CONTINUE ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = DEBUGGING_CONTINUE_ACTION;
			writeUART(send, 3);
			break;
		}

		if(!debugger_breaked) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_BREAKED;
			send[2] = DEBUGGING_CONTINUE_ACTION;
			writeUART(send, 3);
			break;
		}

		// Tell gdb to continue
		char* gdbsend = "continue\n";
		fullWrite(debugger_wfd, (uint8_t*) gdbsend, strlen(gdbsend));

		debugger_breaked = 0;

		break;
	} case DEBUGGING_ADD_BREAKPOINT_ACTION: {
		printf("DEBUGGING ADD BREAKPOINT ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = DEBUGGING_ADD_BREAKPOINT_ACTION;
			writeUART(send, 3);
			break;
		}

		if(!debugger_breaked) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_BREAKED;
			send[2] = DEBUGGING_ADD_BREAKPOINT_ACTION;
			writeUART(send, 3);
			break;
		}

		uint16_t lineNumber = (command[35] << 8) | command[36];
		lineNumber += 3; // consider imports
		// Tell gdb to set breakpoint
		char gdbsend[128];
		int sendLen = snprintf(gdbsend, 128, "echo _Hedgehog_:ignore\\n\nbreak %d\necho _Hedgehog_\\n\n", lineNumber);
		fullWrite(debugger_wfd, (uint8_t*) gdbsend, sendLen);

		break;
	} case DEBUGGING_REMOVE_BREAKPOINT_ACTION: {
		printf("DEBUGGING REMOVE BREAKPOINT ACTION\n"); // <---

		if(program_pid < 0) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_RUNNING;
			send[2] = DEBUGGING_REMOVE_BREAKPOINT_ACTION;
			writeUART(send, 3);
			break;
		}

		if(!debugger_breaked) {
			uint8_t send[3];
			send[0] = ERROR_ACTION;
			send[1] = ERRORCODE_PROGRAM_IS_NOT_BREAKED;
			send[2] = DEBUGGING_REMOVE_BREAKPOINT_ACTION;
			writeUART(send, 3);
			break;
		}

		uint16_t lineNumber = (command[35] << 8) | command[36];
		lineNumber += 3; // consider imports
		// Tell gdb to set breakpoint
		char gdbsend[20];
		int sendLen = snprintf(gdbsend, 20, "clear %d\n", lineNumber);
		fullWrite(debugger_wfd, (uint8_t*) gdbsend, sendLen);

		break;
	} default:
		break;
	}

	free(command);
	return 0;

}

void uprog_cmd_received(uint8_t* command, uint32_t length) {
	switch(command[0]) {
	case CUSTOM_DATA_AVAILABLE_REQUEST_SWCINTERN: {
		uint8_t reply[5];
		reply[0] = CUSTOM_DATA_AVAILABLE_REPLY_SWCINTERN;
		int size = availableFIFO(customDataBuffer);
		reply[1] = ((size >> 24) & 0xFF);
		reply[2] = ((size >> 16) & 0xFF);
		reply[3] = ((size >> 8) & 0xFF);
		reply[4] = (size & 0xFF);
		if(axcpEncodeAndSend(uprog_cmd_wfd, reply, 5) == -1)
			bailOut("Unable to write to pipe\n");
		return;
	} case READ_CUSTOM_DATA_REQUEST_SWCINTERN: {
		uint32_t size = (command[1] << 24) | (command[2] << 16) | (command[3] << 8) | command[4];
		uint8_t reply[size + 1];
		reply[0] = READ_CUSTOM_DATA_REPLY_SWCINTERN;
		uint32_t i;
		for(i=0; i<size; i++) {
			if(readFIFO(reply + 1 + i, customDataBuffer) == -1) {
				reply[1 + i] = 0;
			}
		}
		if(axcpEncodeAndSend(uprog_cmd_wfd, reply, size + 1) == -1)
			bailOut("Unable to write to pipe\n");
		return;
  } case SEND_CUSTOM_DATA_ACTION_SWCINTERN: {
		uint32_t size = length - 1;
		uint8_t send[size + 35];
    send[0] = EXECUTION_DATA_ACTION;
		memcpy(send + 1, currName, 32);
		send[33] = (currVersion >> 8) & 0xFF;
		send[34] = currVersion & 0xFF;
    memcpy(send + 35, command + 1, size);
    writeUART(send, size + 35);
		return;
	} case ANALOG_SENSOR_REQUEST:
		replyOpcode = ANALOG_SENSOR_REPLY;
		replyPort = command[1];
		break;
	case DIGITAL_SENSOR_REQUEST:
		replyOpcode = DIGITAL_SENSOR_REPLY;
		replyPort = command[1];
		break;
	case MOTOR_POSITION_REQUEST:
		replyOpcode = MOTOR_POSITION_REPLY;
		replyPort = command[1];
		break;
	case MOTOR_VELOCITY_REQUEST:
		replyOpcode = MOTOR_VELOCITY_REPLY;
		replyPort = command[1];
		break;
	case CONTROLLER_BATTERY_CHARGE_REQUEST:
		replyOpcode = CONTROLLER_BATTERY_CHARGE_REPLY;
		break;
	case CONTROLLER_BATTERY_CHARGING_STATE_REQUEST:
		replyOpcode = CONTROLLER_BATTERY_CHARGING_STATE_REPLY;
		break;
	case PHONE_BATTERY_CHARGE_REQUEST:
		replyOpcode = PHONE_BATTERY_CHARGE_REPLY;
		break;
	case PHONE_BATTERY_CHARGING_STATE_REQUEST:
		replyOpcode = PHONE_BATTERY_CHARGING_STATE_REPLY;
		break;
	default:
		break;
	}
	writeUART(command, length);
}

void uprog_out_received(uint8_t *text, uint32_t length) {
	uint8_t send[35 + length];
	send[0] = EXECUTION_PRINTOUT_ACTION;
	memcpy(send + 1, currName, 32);
	send[33] = (currVersion >> 8) & 0xFF;
	send[34] = currVersion & 0xFF;
	memcpy(send + 35, text, length);
	writeUART(send, 35 + length);
}

void gdb_out_received_command(char **lines, uint32_t numberOfLines) {
	uint32_t j;
	for(j=0; j<numberOfLines; j++)
		printf("%s\n", lines[j]);


	if(lines[0][10] != ':')
		bailOut("Wrong magic character in gdb injected command\n");
	int commandLen = strlen(lines[0]) - 11;
	// include \0 terminator
	char command[commandLen + 1];
	memcpy(command, lines[0] + 11, commandLen + 1);
	if(strcmp(command, "breaked") == 0) {
		printf("GDB INJECTED BREAKED COMMAND\n"); // <---

		debugger_breaked = 1;

		uint16_t lineNumber = (uint16_t) (atoi(lines[2])) - 3; // minus 2 because of added includes
		uint32_t locLen = 0;
		uint32_t i;
		for(i=3; i < numberOfLines - 1; i++)
			locLen += strlen(lines[i]) + 1; // include \n separator
		locLen--; // exclude last \n separator
		uint8_t send[37 + locLen];
		send[0] = DEBUGGING_BREAKED_ACTION;
		memcpy(send + 1, currName, 32);
		send[33] = (currVersion >> 8) & 0xFF;
		send[34] = currVersion & 0xFF;
		send[35] = (lineNumber >> 8) & 0xFF;
		send[36] = lineNumber & 0xFF;
		int curPos = 37;
		for(i=3; i < numberOfLines - 1; i++) {
			memcpy(send + curPos, lines[i], strlen(lines[i]));
			curPos += strlen(lines[i]);
			if(i < numberOfLines - 2) {
				send[curPos] = '\n';
				curPos++;
			}
		}
		writeUART(send, 37 + locLen);
	} else if(strcmp(command, "ignore") == 0) {
		printf("GDB INJECTED IGNORE COMMAND\n"); // <---
	} else {
		bailOut("Unknown injected gdb command.");
	}

	uint32_t i;
	for(i=0; i<numberOfLines; i++)
		free(lines[i]);
	free(lines);
}

int main() {

	printf("Hedgehog successfully started.\n");


	setvbuf(stdout, NULL, _IONBF, 0);

	// Open communication pipes to the debugger process
	int rpipe[2];
	int wpipe[2];
	if(pipe(rpipe) < 0)
		bailOut("Failed to open debugger read pipe\n");
	if(pipe(wpipe) < 0)
		bailOut("Failed to open debugger write pipe\n");

	// Start child process
	int pid = fork();
	if(pid < 0) {
		bailOut("Failed to fork\n");
	} else if(pid == 0) {
		// Redirect child fds and start user program
		close(rpipe[0]);
		close(wpipe[1]);
		if(dup2(rpipe[1], STDOUT_FILENO) == -1)
			bailOut("Debugger write dup2 failed\n");
		if(dup2(wpipe[0], STDIN_FILENO) == -1)
			bailOut("Debugger read dup2 failed\n");
		close(rpipe[1]);
		close(wpipe[0]);
		execlp("gdb", "gdb", "-q", NULL);
		bailOut("Debugger exec fail\n");
	}

	// init communication fds and global variables
	close(rpipe[1]);
	close(wpipe[0]);
	debugger_wfd = wpipe[1];
	pfds[3].fd = rpipe[0];
	printf("Debugger successfully started.\n");

	// pfds[0].fd = open("./input", O_RDONLY);
	// pfds[0].fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	pfds[0].fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
	if(pfds[0].fd == -1)
		bailOut("Unable to open uart input\n");

	struct termios options;
	tcgetattr(pfds[0].fd, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(pfds[0].fd, TCIFLUSH);
	tcsetattr(pfds[0].fd, TCSANOW, &options);

	pfds[0].events = POLLIN;
	pfds[0].revents = 0;
	pfds[1].fd = -1;
	pfds[1].events = POLLIN;
	pfds[1].revents = 0;
	pfds[2].fd = -1;
	pfds[2].events = POLLIN;
	pfds[2].revents = 0;
	pfds[3].events = POLLIN;
	pfds[3].revents = 0;
	pfds[4].fd = STDIN_FILENO;
	pfds[4].events = POLLIN;
	pfds[4].revents = 0;

  uint8_t stdin_buffer[256];
	uint8_t uprog_out_buffer[512];

  uint8_t send[1];
	send[0] = HW_CONTROLLER_TYPE_REQUEST;
	writeUART(send, 1);

	while(1) {
		uint32_t rx_length = 0, uprog_cmd_length = 0;
		int stdin_length = -1, uprog_out_length = 0, gdb_out_length = 0;
		uint8_t *rx_buffer, *uprog_cmd_buffer;
		char **gdb_out_buffer;

		if(program_pid > -1) {
			int status;
			int result = waitpid(program_pid, &status, WNOHANG);
			if(result == -1)
				bailOut("Couldn't wait for child\n");
			if(result > 0 && WIFEXITED(status) != 0) {
				printf("Program exited with status %d\n", status); // <---
				uint8_t send[39];
				send[0] = EXECUTION_DONE_ACTION;
				memcpy(send + 1, currName, 32);
				send[33] = (currVersion >> 8) & 0xFF;
				send[34] = currVersion & 0xFF;
				int retVal = WEXITSTATUS(status);
				send[35] = (retVal >> 24) & 0xFF;
				send[36] = (retVal >> 16) & 0xFF;
				send[37] = (retVal >> 8) & 0xFF;
				send[38] = retVal & 0xFF;
				writeUART(send, 39);
				destroyFIFO(customDataBuffer);
				customDataBuffer = NULL;
				program_pid = -1;
				debugger_attached = 0;
				debugger_breaked = 0;
			} else if(result > 0 && WIFSIGNALED(status) != 0) {
				// Check if terminating signal was SIGINT
				if(WTERMSIG(status) == SIGTERM) {
					printf("Program signaled via SIGTERM!\n"); // <---
					uint8_t send[35];
					send[0] = EXECUTION_STOPPED_ACTION;
					memcpy(send + 1, currName, 32);
					send[33] = (currVersion >> 8) & 0xFF;
					send[34] = currVersion & 0xFF;
					writeUART(send, 35);
					program_pid = -1;
					debugger_attached = 0;
					debugger_breaked = 0;
				}
			}
		}

		if(restart) {
			int result = executeProgram(currName, currVersion);
			if(result == -2) {
				uint8_t send[3];
				send[0] = ERROR_ACTION;
				send[1] = ERRORCODE_PROGRAM_NOT_FOUND;
				send[2] = EXECUTION_RESTART_ACTION;
				writeUART(send, 3);
				restart = 0;
			} else if(result == 0) {
				uint8_t send[35];
				send[0] = EXECUTION_STARTED_ACTION;
				memcpy(send + 1, currName, 32);
				send[33] = (currVersion >> 8) & 0xFF;
				send[34] = currVersion & 0xFF;
				writeUART(send, 35);
				restart = 0;
			}
			// If result = -1, the program hasn't shutdown completely yet - try again later
		}

		pfds[0].revents = 0;
		pfds[1].revents = 0;
		pfds[2].revents = 0;
		pfds[3].revents = 0;
		pfds[4].revents = 0;
		int res = poll(pfds, 5, 0);
		if(res < 0) {
			bailOut("Failed to poll\n");
		}
		if(pfds[0].revents > 0) {
			if((pfds[0].revents & POLLIN) > 0) {
				int result = axcpReceiveAndDecode(pfds[0].fd, &rx_buffer, &rx_length);
				if(result == -1)
					bailOut("UART receive failed\n");
				if(result == -2) {
				        printf("Unknown opcode from UART %d\n", rx_buffer[0]);
					//uint8_t send[3];
					//send[0] = ERROR_ACTION;
					//send[1] = ERRORCODE_UNSPECIFIED_OPCODE;
					//send[2] = rx_buffer[0];
					//writeUART(send, 3);
					//usleep(600000);
					//uint8_t buf[10];
					//while(read(pfds[0].fd, buf, 10) > 0);
					//printf("Done waiting and flushing\n");
				}
			} else {
				bailOut("Error flag after poll uart\n");
			}
		}
		if(pfds[1].revents > 0) {
			if((pfds[1].revents & POLLIN) > 0) {
				int result = axcpReceiveAndDecode(pfds[1].fd, &uprog_cmd_buffer, &uprog_cmd_length);
				if(result == -1)
					bailOut("UART receive failed\n");
				if(result == -2)
					bailOut("Unknown opcode from pipe\n");
			} else if(program_pid == -1) {
        destroyFIFO(customDataBuffer);
			  customDataBuffer = NULL;	
				pfds[1].fd = -1;
				uprog_cmd_wfd = -1;
				printf("Cmd pipes have been closed\n");
			}
			// It is assumed that no error flag happens for the pipe connection
		}
		if(pfds[2].revents > 0) {
			if((pfds[2].revents & POLLIN) > 0) {
				uprog_out_length = read(pfds[2].fd, uprog_out_buffer, 512);
				if(uprog_out_length < 0)
					bailOut("Unable to read from out buffer\n");
			} else if(program_pid == -1) {
				pfds[2].fd = -1;
				printf("Out pipe has been closed\n");
			}
			// It is assumed that no error flag happens for the pipe connection
		}
		if(pfds[3].revents > 0) {
			if((pfds[3].revents & POLLIN) > 0) {
				gdb_out_buffer = (char**) malloc(sizeof(char*));
				char *firstLine = (char*) malloc(256);
				gdb_out_buffer[0] = firstLine;
				int curPos = 0, res = 0;
				do {
					res = fullRead(pfds[3].fd, (uint8_t*) (gdb_out_buffer[0] + curPos), 1);
					if(res == -1)
						bailOut("Failed to read from gdb\n");
					curPos++;
				} while(gdb_out_buffer[0][curPos - 1] != '\n');
				gdb_out_buffer[0][curPos - 1] = '\0';
				if(curPos >= 10 && strncmp(gdb_out_buffer[0], "Breakpoint", 10) == 0) {
					char *gdbsend = "echo _Hedgehog_:breaked\\n\nframe\ninfo locals\necho _Hedgehog_\\n\n";
					fullWrite(debugger_wfd, (uint8_t*) gdbsend, strlen(gdbsend));
					free(gdb_out_buffer[0]);
					free(gdb_out_buffer);

					printf("Recognized Breakpoint in gdb output and injected command\n"); // <---

					// Command injection :D
				} else if(curPos >= 10 && strncmp(gdb_out_buffer[0], "_Hedgehog_", 10) == 0) {
					printf("Recognized injected gdb command, beginning to capture...\n"); // <---
					int curLine = 1;
					while(1) {
						char *nextLine = (char*) malloc(256);
						gdb_out_buffer = (char**) realloc(gdb_out_buffer, (curLine+1) * sizeof(char*));
						gdb_out_buffer[curLine] = nextLine;
						curPos = 0, res = 0;
						do {
							res = fullRead(pfds[3].fd, (uint8_t*) (gdb_out_buffer[curLine] + curPos), 1);
							if(res == -1)
								bailOut("Failed to read from gdb\n");
							curPos++;
						} while(gdb_out_buffer[curLine][curPos - 1] != '\n');
						gdb_out_buffer[curLine][curPos - 1] = '\0';
						if (curPos == 11 && strncmp(gdb_out_buffer[curLine], "_Hedgehog_", 10) == 0) {
							printf("Capture done!\n"); // <---
							gdb_out_length = curLine + 1;
							break;
						}
						curLine++;
					}
				} else {
					printf("gdb line ignored.\n");  //<----
					free(gdb_out_buffer[0]);
					free(gdb_out_buffer);
				}
			}
			// It is assumed that no error flag happens for the pipe connection
		}
		if(pfds[4].revents > 0) {
			if((pfds[4].revents & POLLIN) > 0) {
				do {
					stdin_length++;
					if(fullRead(pfds[4].fd, stdin_buffer + stdin_length, 1) == -1)
						bailOut("Unable to read from stdin\n");
				} while(stdin_buffer[stdin_length] != '\n');
				stdin_buffer[stdin_length] = '\0';
			}
		}


		if(rx_length > 0) {
      printf("Received from UART opcode %d: ", rx_buffer[0]);
      uint32_t i;
      for(i=1; i<rx_length; i++)
        printf("%d,", rx_buffer[i]);
      printf("\n");
			if(uart_cmd_received(rx_buffer, rx_length) == 1)
				break;
		}

		if(uprog_cmd_length > 0) {
			printf("Received from uprog cmd opcode %d: ", uprog_cmd_buffer[0]);
      uint32_t i;
      for(i=1; i<uprog_cmd_length; i++)
        printf("%d,", uprog_cmd_buffer[i]);
      printf("\n");
      uprog_cmd_received(uprog_cmd_buffer, uprog_cmd_length);
		}

		if(uprog_out_length > 0) {
			uprog_out_received(uprog_out_buffer, uprog_out_length);
		}

		if(gdb_out_length > 0) {
			gdb_out_received_command(gdb_out_buffer, gdb_out_length);
		}

		if(stdin_length > 0) {
			uint8_t stdin_send[256];
			char* current = strtok((char*) stdin_buffer, " ");
			int i;
			for(i=0; current != NULL; i++) {
				stdin_send[i] = (uint8_t) atoi(current);
				current = strtok(NULL, " ");
			}
                        if(payloadLength(stdin_send[0]) == -2)
				fullWrite(pfds[0].fd, stdin_send, i);
			else
				writeUART(stdin_send, i);
		}

	}

	if(pfds[0].fd != -1)
		close(pfds[0].fd);
	if(pfds[1].fd != -1)
		close(pfds[1].fd);
	if(pfds[2].fd != -1)
		close(pfds[2].fd);
	if(pfds[3].fd != -1)
		close(pfds[3].fd);
	if(uprog_cmd_wfd != -1)
		close(uprog_cmd_wfd);

	return EXIT_SUCCESS;
}
