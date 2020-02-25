#include <stdio.h>
#include <math.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>
#include "sys/time.h"

#include "actuator.h"

#define fx_t int32_t
#define FX_FRAC 8
#define fx_make(a)  ((fx_t)((a) * ((int)1<<FX_FRAC)))


int usb;
pthread_t serial_recv_thread;
FILE *logfile;

typedef enum
{ Parser_P = 0,
  Parser_A = 1,
  Parser_A_val = 2,
  Parser_D = 3,
  Parser_D_val = 4,
  Parser_F_val = 5 
} ParserState;

static ParserState parser = Parser_P;
int parser_axis;
int parser_axis_value;
int parser_axis_flags;

typedef enum {
	K_P = 0,
	K_D = 1,
	K_I = 2,
	K_PWR = 3,
	K_STALL_THOLD_HOME = 4,
	K_STALL_THOLD_RUN = 5,
	K_IDLE_TIMEOUT = 6,
	K_I_LIMIT = 7
} act_constant_t;

void send_ping(void);
void send_constant_PID(act_constant_t c, float k);
void send_constant(act_constant_t c, int k);


#define EXTENT ((250/5)*(1200/2))

void *recv_serial_response_thread(void *);
actuator_state act_state[6];


char servo_state[16][32] = { "Servo_Init", "Servo_FindHome", "Servo_Run", "Servo_Error", "Servo_Stop", "Servo_Stop_NoHome" };

void setup_serial(void) {
	// serial port
	usb = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NDELAY);
	struct termios tty;

	memset (&tty, 0, sizeof tty);

	//memset (fs, sizeof(struct f_state) * 6 * 3, 0);
	
	cfsetospeed(&tty, (speed_t)B115200);
	cfsetispeed(&tty, (speed_t)B115200);
	tty.c_cflag &= ~PARENB & ~CSTOPB & ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5;
	tty.c_cflag |= CREAD | CLOCAL;

	cfmakeraw(&tty);
	tcflush(usb, TCIFLUSH);
	tcsetattr(usb, TCSANOW, &tty);

	pthread_create(&serial_recv_thread, NULL, recv_serial_response_thread, (void *) NULL);

	logfile = fopen("log.csv", "w");

	memset(act_state, 0, sizeof(act_state));

	// init actuators
	int nr[6];
	int i;
	for (i=0; i<6; i++) nr[i] = act_state[i].num_responses;
	send_ping();
	int act_found = 0;
	for (i=0; i<1; i++) { // FIXME!
	//for (;;) {
		int j;
		act_found = 0;
		for (j=0; j<6; j++) if (nr[j] != act_state[j].num_responses) act_found++;
		if (act_found == 6) {printf("Found all 6!\n"); break; }
		usleep(1000);
		send_ping();
	}
	sleep(1);
	//exit(1);
	
	printf("Actuators found %d\n", act_found);
	for (i=0; i<6; i++) {
		if (nr[i] != act_state[i].num_responses) {
			printf("Actuator %d :\tState=%s\tPos=%d\n", i, servo_state[FLAGS_STATE(act_state[i].last_flags)], act_state[i].actual_pos); 
			act_state[i].active = 1;
		} else {
			printf("Actuator %d :\tNOT FOUND\n", i);
			act_state[i].active = 0;
		}
	}

	//exit(1);
	
	sleep(1);

	printf("Setting constants...\n");
	send_constant(K_PWR, 300);	

	sleep(1);

	send_constant_PID(K_P, 0.20);

	sleep(1);

	send_constant_PID(K_D, 0.1);

	sleep(1);

	send_constant_PID(K_I, 0.002);

	sleep(1);


	//if (act_found != 6) exit(0);	
	
	//exit(0);

	int j;
	int ready = 1;
	for (j=0; j<6; j++) ready &= ((!act_state[j].active) || (FLAGS_STATE(act_state[j].last_flags)==4 /*Servo_Stop*/)) ;
	if (ready) {
		printf("No need to home...\n");	
		return;
	}
	// Need to find home


	printf("Homing...\n");
	send_home();
	// wait till all are home (20 sec)
	for (i=0; i<20; i++) {
		int j;
		int ready = 1;
		for (j=0; j<6; j++) ready &= ((!act_state[j].active) || (FLAGS_STATE(act_state[j].last_flags)==4 /*Servo_Stop*/)) ;
		if (i>4 && ready) break;
		send_ping();
		sleep(1);
	}
	for (i=0; i<6; i++) {
		if (act_state[i].active) {
			printf("Actuator %d :\tState=%s\tPos=%d\n", i, servo_state[FLAGS_STATE(act_state[i].last_flags)], act_state[i].actual_pos); 
		} else {
			printf("Actuator %d :\tNOT FOUND\n", i);
		}
	}
}

void parser_recv(char c) {
	if (c == 'P') {
		parser = Parser_A;
	}
	
	switch (parser) {
		case Parser_P:
		break;
		case Parser_A:
		if (c=='A') parser = Parser_A_val;
		break;
		case Parser_A_val:
		if (isdigit(c)) {
			parser_axis = c - '0';
			parser = Parser_D;
		} else parser = Parser_A;
		case Parser_D:
		if (c=='D') {
			parser = Parser_D_val;
			parser_axis_value = 0;
		}
		break;
		case Parser_D_val:
		if (isdigit(c)) {
			parser_axis_value*=10;
			parser_axis_value += c - '0';
		} else {
			if (c=='A')
				parser = Parser_A_val;
			else if (c=='F') {
				parser = Parser_F_val;
				parser_axis_flags = 0;
			} else
				parser = Parser_A;
		}
		break;
		case Parser_F_val:
		if (isdigit(c)) {
			parser_axis_flags *=10;
			parser_axis_flags += c - '0';
		} else {
			if (c=='A')
				parser = Parser_A_val;
			else
				parser = Parser_A;

			//printf("RECEIVED: Axis=%d value=%d State=%s\n", parser_axis, parser_axis_value, servo_state[FLAGS_STATE(parser_axis_flags)]);
			//if (parser_axis_flags & FLAG_BRAKE_LIMIT) printf("Axis %d reached braking limit!\n", parser_axis);

			if (parser_axis >=0 && parser_axis < 6) {
				act_state[parser_axis].actual_pos = (int)((float)parser_axis_value * ((float)((1<<16)-1)/(float)EXTENT));
				gettimeofday(&(act_state[parser_axis].last_response), NULL);
				act_state[parser_axis].last_flags = parser_axis_flags;
				act_state[parser_axis].num_responses++; 
			}
			fprintf(logfile, "%d\t%d\n", act_state[2].actual_pos, act_state[2].cmd_pos);
			fflush(logfile);
		}
		break;
	}
}

void *recv_serial_response_thread(void * param) {
	ssize_t br;
	char cmd[256];
	while (1) {
		bzero(cmd, 256);
		br = read(usb, cmd, sizeof(cmd));
		if (br>0) {
			//printf("READ FROM SERIAL: %s\n", cmd);
			int i;	
			// parse response
			for (i=0; i<br; i++) {
				parser_recv(cmd[i]);
			}
		}
	}
}

void send_serial_command(int *len) {
	int i;
	char cmd[64];
	for (i=0; i<6; i++) {
		if (len[i]<8000) len[i] = 8000;
		if (len[i]>((1<<16)-8000)) len[i] = ((1<<16)-8000);
		act_state[i].cmd_pos = len[i];
		
		if (act_state[i].active && FLAGS_STATE(act_state[i].last_flags) != 2 && FLAGS_STATE(act_state[i].last_flags) != 4) exit(1);

		//printf("Commanding %d to %d\n", i, len[i]);
	}

	sprintf(cmd, "PA0D%dA1D%dA2D%dA3D%dA4D%dA5D%dA", len[0], len[1], len[2], len[3], len[4], len[5]); // FIXME! order
	//sprintf(cmd, "PA1D%dA", len[1]);
        //printf("sending %s\n", cmd);
	size_t remain = strlen(cmd);
	size_t start = 0;

	while (remain>0) {
		start = write(usb, cmd + start, remain);
		remain -= start;
	}
}

void send_home(void) {
	char cmd[64];
	sprintf(cmd, "H");
	write(usb, cmd, strlen(cmd));
}

void send_ping(void) {
	char cmd[64];
	sprintf(cmd, "I");
	//printf("ping...\n");
	write(usb, cmd, strlen(cmd));
}

void send_constant_PID(act_constant_t c, float k) {
	char cmd[64];
	if (c<K_P || c>K_I) return;
	fx_t val;
	val = fx_make(k);
	sprintf(cmd, "PK%dD%dA\n", c, val);
	write(usb, cmd, strlen(cmd));
}

void send_constant(act_constant_t c, int k) {
	char cmd[64];
	if (c<K_PWR || c>K_I_LIMIT) return;
	sprintf(cmd, "PK%dD%dA\n", c, k);
	write(usb, cmd, strlen(cmd));
}


