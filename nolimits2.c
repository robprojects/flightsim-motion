#define PORT 15152

#define BUFSIZE 256

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <math.h>
#include "nolimits2.h"
#include "motion.h"

typedef struct {
	int sockfd;
	pthread_t udp_thread;
	int run;

	float theta;
	float psi;
	float phi;
	float side;
	float heave; //nrml
	float back; //axil
	int paused;
} nl2state_t;

typedef struct nl2hdr {
	uint8_t start; // 'N'
	uint16_t type;
	uint32_t req;
	uint16_t size;
	char end; // 'L'
} __attribute__ ((packed)) nl2hdr_t;

// id = 5

// id = 6
typedef struct nl2telemetry {
	int32_t state;
	int32_t frame;
	int32_t viewmode;
	int32_t coaster;
	int32_t style;
	int32_t train;
	int32_t car;
	int32_t seat;
// floats (be)
	int32_t speed;
	int32_t pos_x;
	int32_t pos_y;
	int32_t pos_z;
	int32_t rotq_x;
	int32_t rotq_y;
	int32_t rotq_z;
	int32_t rotq_w;
	int32_t g_x;
	int32_t g_y;
	int32_t g_z;
} __attribute__ ((packed)) nl2telemetry_t;

float tohfloat(int32_t in) {
	int32_t val = ntohl(in);
	float out = *(float *)&val;
	return out;
}

void *nl_run(void *state);

void *nolimits2_init(char *server, int sample) {
	nl2state_t *state = malloc(sizeof(nl2state_t));

	state->psi = 0.0;
	state->phi = 0.0;
	state->theta = 0.0;

	struct sockaddr_in serv;

	state->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	state->run = 1;
	
	bzero((char *) &serv, sizeof(serv));
	serv.sin_family = AF_INET;
	serv.sin_port = htons(PORT);
	inet_aton(server, &(serv.sin_addr));

	connect(state->sockfd, &serv, sizeof(serv));

	pthread_create(&(state->udp_thread), NULL, nl_run, (void *) state);	

	return state;
}

int nolimits2_readnext(void *sstate, double *Faa, double *Oaa, int *valid) {

	nl2state_t *state = (nl2state_t*)sstate;

	Oaa[2] = state->theta; Oaa[1] = state->psi; Oaa[0] = state->phi;
	Faa[0] = /*side*/state->side; Faa[1] = /*axil*/state->back; Faa[2] = state->heave /*nrml*/;


	*valid = !state->paused;
	return 0;
}


void *nl_run(void *state_v) {

	float last_w=0.0, last_x=0.0, last_y=0.0, last_z=0.0;
	
	nl2state_t *state = (nl2state_t*)state_v;

	nl2hdr_t *reqmsg = malloc(sizeof(nl2hdr_t));
	reqmsg->start = 'N';
	reqmsg->type = htons(5);
	reqmsg->req = 0;
	reqmsg->size = 0;
	reqmsg->end = 'L';

	char *buf = malloc(BUFSIZE);

	int req = 0;

	while (state->run) {

	reqmsg->start = 'N';
	reqmsg->type = htons(5);
	reqmsg->req = htonl(req++);
	reqmsg->size = 0;
	reqmsg->end = 'L';


	write(state->sockfd,reqmsg,sizeof(nl2hdr_t));
	

	int msg_size=-1;
	int n, size=0;
	while (size < BUFSIZE) {
		n = read(state->sockfd,buf + size,BUFSIZE);
		size += n;
		if (n>sizeof(nl2hdr_t)) {
			if (((nl2hdr_t *)buf)->start == 'N') {
				msg_size = ntohs(((nl2hdr_t *)buf)->size);
				printf("type=%d msg_size=%d\n", ((nl2hdr_t *)buf)->type, msg_size);
			}
		}
		if (msg_size != -1 && size >= msg_size) {
			break;
		}
		printf("read %d\n", size);
	}
	nl2hdr_t *hdr = buf;
	nl2telemetry_t *msg = &(hdr->end);

	/*printf("msg received size=%d magic=%c\n", ntohs(hdr->size), hdr->start);*/
	int gstate = ntohl(msg->state);
/*
	if (gstate & 0x1) printf("play mode\n");
	if (gstate & 0x2) printf("braking\n");
	if (gstate & 0x4) printf("paused\n");
*/
	state->paused = !(gstate&0x1) || (gstate&0x4);

	printf("g_x=%f\n", tohfloat(msg->g_x));	
	printf("g_y=%f\n", tohfloat(msg->g_y));	
	printf("g_z=%f\n", tohfloat(msg->g_z));	

	printf("rotq_x=%f\n", tohfloat(msg->rotq_x));	
	printf("rotq_y=%f\n", tohfloat(msg->rotq_y));	
	printf("rotq_z=%f\n", tohfloat(msg->rotq_z));	
	printf("rotq_w=%f\n", tohfloat(msg->rotq_w));

	float q_x = tohfloat(msg->rotq_x);
	float q_y = tohfloat(msg->rotq_y);
	float q_z = tohfloat(msg->rotq_z);
	float q_w = tohfloat(msg->rotq_w);


	state->side = -tohfloat(msg->g_x)*9.8; // side
	state->heave = tohfloat(msg->g_y)*9.8; // heave
	state->back = -tohfloat(msg->g_z)*9.8; // back

	//FIXME
	/*state->side = 0.0;
	state->heave = 0.0;
	state->back = 0.0;*/

	// conjugate last
	last_x = -last_x;
	last_y = -last_y;
	last_z = -last_z;

	// multiply current with last
	float x = q_w * last_x + q_x * last_w + q_z * last_z - q_z * last_y;
	float y = q_w * last_y + q_y * last_w + q_x * last_x - q_x * last_z;
	float z = q_w * last_z + q_z * last_w + q_x * last_y - q_y * last_x;
	float w = q_w * last_w + q_x * last_x + q_y * last_y - q_z * last_z;


	// roll (x-axis rotation)
	float sinr_cosp = +2.0 * (w * x + y * z);
	float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
	float roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float pitch;
	float sinp = +2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	float siny_cosp = +2.0 * (w * z + x * y);
	float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);  
	float yaw = atan2(siny_cosp, cosy_cosp);


	state->psi += (yaw/M_PI)*180.0; //(pitch/M_PI)*180.0; // yaw
	state->phi += (pitch/M_PI)*180.0; // roll
	state->theta += (roll/M_PI)*180.0; // (yaw/M_PI)*180.0; // pitch
	
	printf("w=%f x=%f y=%f z=%f sinp=%f pitch=%f roll=%f yaw=%f\n", q_w, q_x, q_y, q_z, sinp, state->psi, state->phi, state->theta);

	last_w = q_w; last_x = q_x; last_y = q_y; last_z = q_z;

	usleep((int)(1000000.0/SAMPLE));
	}
}


/*
int main(void) {
	nl2state_t *state = malloc(sizeof(nl2state_t));

	struct sockaddr_in serv;

	state->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
	bzero((char *) &serv, sizeof(serv));
	serv.sin_family = AF_INET;
	serv.sin_port = htons(PORT);
	inet_aton("192.168.1.66", &serv.sin_addr.s_addr);

	connect(state->sockfd, &serv, sizeof(serv));
	
	nl2hdr_t *reqmsg = malloc(sizeof(nl2hdr_t));
	reqmsg->start = 'N';
	reqmsg->type = htons(5);
	reqmsg->req = 0;
	reqmsg->size = 0;
	reqmsg->end = 'L';

	char *buf = malloc(BUFSIZE);

	while (1) {
	write(state->sockfd,reqmsg,sizeof(nl2hdr_t));
	

	int msg_size=-1;
	int n, size=0;
	while (size < BUFSIZE) {
		n = read(state->sockfd,buf + size,BUFSIZE);
		size += n;
		if (n>sizeof(nl2hdr_t)) {
			if (((nl2hdr_t *)buf)->start == 'N') {
				msg_size = ntohs(((nl2hdr_t *)buf)->size);
				printf("type=%d msg_size=%d\n", ((nl2hdr_t *)buf)->type, msg_size);
			}
		}
		if (msg_size != -1 && size >= msg_size) {
			break;
		}
		printf("read %d\n", size);
	}
	nl2hdr_t *hdr = buf;
	nl2telemetry_t *msg = &(hdr->end);

	printf("msg received size=%d magic=%c\n", ntohs(hdr->size), hdr->start);
	int gstate = ntohl(msg->state);
	if (gstate & 0x1) printf("play mode\n");
	if (gstate & 0x2) printf("braking\n");
	if (gstate & 0x4) printf("paused\n");

	state->paused = !(gstate&0x1) || (gstate&0x4);

	printf("g_x=%f\n", tohfloat(msg->g_x));	
	printf("g_y=%f\n", tohfloat(msg->g_y));	
	printf("g_z=%f\n", tohfloat(msg->g_z));	

	printf("rotq_x=%f\n", tohfloat(msg->rotq_x));	
	printf("rotq_y=%f\n", tohfloat(msg->rotq_y));	
	printf("rotq_z=%f\n", tohfloat(msg->rotq_z));	
	printf("rotq_w=%f\n", tohfloat(msg->rotq_w));	

	state->side = tohfloat(msg->g_x)*9.8; // side
	state->heave = tohfloat(msg->g_y)*9.8; // heave
	state->back = tohfloat(msg->g_z)*9.8; // back

	usleep((int)(1000000.0/SAMPLE));
	}
}
*/

