#define PORT 8888

#define BUFLEN 1024

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "motion.h"

typedef struct {
	double x,y,z;
	double x_rot, y_rot, z_rot;
	pthread_t udp_thread;
	int run;
	int s;
	//struct sockaddr_in si;
	//int slen;
	char *server;
	int sample;
} rf2_udp_state_t;

void *rf2_udp_run(void *state);

static int paused = 1;

void *rf2_init(char *server, int sample) {
	rf2_udp_state_t *state = malloc(sizeof(rf2_udp_state_t));
	state->server = strdup(server);
	state->sample = sample;

	pthread_create(&(state->udp_thread), NULL, rf2_udp_run, (void *) state);	
	return state;
}

int rf2_readnext(void *sstate, double *Faa, double *Oaa, int *valid) {
	float theta, psi, phi, y, z, x;
	rf2_udp_state_t *udp_state=(rf2_udp_state_t *) sstate;
/*
	theta = udp_state->theta;
	psi=udp_state->psi;
	phi=udp_state->phi;
	y=udp_state->a_side;
	z=udp_state->a_nrml;
	x=udp_state->a_axil;
*/
	double scale = 1.0;

	Oaa[2] = -udp_state->x_rot * scale; /*theta - pitch*/ Oaa[1] = udp_state->y_rot*scale; /*psi - heading */; Oaa[0] = udp_state->z_rot*scale; /* phi - roll*/
	Faa[0] = udp_state->x; Faa[1] = udp_state->z; Faa[2] = udp_state->y;

	//Faa[0] = 0.0; Faa[1] = 0.0; Faa[2] = 0.0;

	//printf("rfactor %f, %f, %f\n", Faa[0], Faa[1], Faa[2]);
	//printf("rfactor %f, %f, %f\n", Oaa[0], Oaa[1], Oaa[2]);

	*valid = !paused;
	
	return 0;
}

void rf2_close(void *sstate) {
	free(sstate);
}


void *rf2_udp_run(void *state_v) {
	rf2_udp_state_t *state = (rf2_udp_state_t *) state_v;
	int recv_len;
	int slen;
	char buf[BUFLEN];
	struct sockaddr_in si;

	state->s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);


	memset((char*) &si, 0, sizeof(si));
	si.sin_family = AF_INET;
	si.sin_port = htons(PORT);
	//inet_aton(state->server, &(si.sin_addr));

	bind(state->s, &si, sizeof(struct sockaddr_in));

	while(state->run) {
		int cnt = 0;
		fd_set readfds;
		struct timeval timeout;
		FD_ZERO(&readfds);
		FD_SET(state->s, &readfds);
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		//printf("waiting...\n");
		int t = select(state->s + 1, &readfds, NULL, NULL, &timeout);
		if (t>0 && FD_ISSET(state->s, &readfds)) {
			recv_len = recvfrom(state->s, buf, BUFLEN, 0, (struct sockaddr *) &si, &slen);
			paused = 0; //FIXME!
			//printf("received %d\n", recv_len);
			int version = buf[0];
			int packet = buf[1];
			int seq = *(short*)(buf + 2);
			int type = buf[4];
			state->x = *(double*)(buf+ 5);
			state->y = *(double*)(buf + 5+8*1);
			state->z = *(double*)(buf + 5+8*2);
			state->x_rot = *(double*)(buf + 5+(8*3));
			state->y_rot = *(double*)(buf + 5+(8*4));
			state->z_rot = *(double*)(buf + 5+(8*5));
		} else {
			printf("timeout...\n");
			paused = 1; // fixme
		}
	}
}
/*
int main(void) {
	xplane_udp_state_t *state = malloc(sizeof(xplane_udp_state_t));
	
	xplane_udp_setup(state, "192.168.1.66", 10);
	state->run=1;
	xplane_udp_run(state);
}*/

