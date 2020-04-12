#define PORT 49000
#define MULTICAST_GROUP "239.255.1.1"
#define MULTICAST_PORT 49707
//#define SERVER "192.168.1.3"

#define BUFLEN 1024

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "xplane.h"
#include "motion.h"

typedef char xchr;
typedef int xint;
typedef float xflt;

// AXIL - backward
// NRML - upward
// SIDE - sideways

typedef enum {GROUNDSPEED=0,
	FNRML_PROP=1, FSIDE_PROP=2, FAXIL_PROP=3,
	FNRML_AERO=4, FSIDE_AERO=5, FAXIL_AERO=6,
	FNRML_GEAR=7, FSIDE_GEAR=8, FAXIL_GEAR=9,
	M_TOTAL=10,
	THETA=11,
	PSI=12,
	PHI=13,
	PAUSE=14} dataref_t;

typedef struct {
	float theta;
	float psi;
	float phi;
	float a_side;
	float a_nrml;
	float a_axil;
	pthread_t udp_thread;
	int run;
	int s;
	//struct sockaddr_in si;
	//int slen;
	char *server;
	int sample;
} xplane_udp_state_t;

void *xplane_udp_run(void *state);

int paused = 1;

static float dataref_val[15];
static char dataref_name[15][400] = {
	"sim/flightmodel/position/groundspeed",
	"sim/flightmodel/forces/fnrml_prop",
	"sim/flightmodel/forces/fside_prop",
	"sim/flightmodel/forces/faxil_prop",
	"sim/flightmodel/forces/fnrml_aero",
	"sim/flightmodel/forces/fside_aero",
	"sim/flightmodel/forces/faxil_aero",
	"sim/flightmodel/forces/fnrml_gear",
	"sim/flightmodel/forces/fside_gear",
	"sim/flightmodel/forces/faxil_gear",
	"sim/flightmodel/weight/m_total",
	"sim/flightmodel/position/theta",
	"sim/flightmodel/position/psi",
	"sim/flightmodel/position/phi",
	"sim/time/paused"
};

struct dref_struct_out {
	xint dref_en;
	xflt dref_flt;
};

void *xplane_init(char *server, int sample) {
	xplane_udp_state_t *state = malloc(sizeof(xplane_udp_state_t));
	// xplane udp
	state->server = strdup(server);
	state->sample = sample;
	xplane_udp_setup(state);

	pthread_create(&(state->udp_thread), NULL, xplane_udp_run, (void *) state);	
	return state;
}

int xplane_readnext(void *sstate, double *Faa, double *Oaa, int *valid) {
	float theta, psi, phi, y, z, x;
	xplane_udp_state_t *udp_state=(xplane_udp_state_t *) sstate;

	theta = udp_state->theta;
	psi=udp_state->psi;
	phi=udp_state->phi;
	y=udp_state->a_side;
	z=udp_state->a_nrml;
	x=udp_state->a_axil;

	Oaa[2] = theta; Oaa[1] = psi; Oaa[0] = phi;
	Faa[0] = y; Faa[1] = x; Faa[2] = z;

	*valid = !paused;
	
	return 0;
}

void xplane_close(void *sstate) {
	free(sstate);
}


//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fltlim(float data, float min, float max)
{
	if (data < min) return min;
	if (data > max) return max;
	return data;
}

//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fallout(float data, float low, float high)
{
	if (data < low) return data;
	if (data > high) return data;
	if (data < ((low + high) * 0.5)) return low;
    return high;
}

//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fltmax2 (float x1,const float x2)
{
	return (x1 > x2) ? x1 : x2;
}




int register_dataref(int s, struct sockaddr_in *si, int nr, char *name, int freq) {
	char buf[BUFLEN];

	struct dref_struct_in {
		xint dref_freq;
		xint dref_en;
		xchr dref_string[400];	
	} rref_in;

	rref_in.dref_freq = freq;
	rref_in.dref_en = nr;
	strncpy(rref_in.dref_string, name, 400);
	memcpy(buf, "RREF\0", 5);
	memcpy(buf+5, &rref_in, sizeof(rref_in));

	sendto(s, (char *) buf, sizeof(rref_in) + 5, 0, (struct sockaddr *) si, sizeof(struct sockaddr_in));
}

int calculate_accel(xplane_udp_state_t *state) {
	float groundspeed = dataref_val[GROUNDSPEED];
	float fnrml_prop = dataref_val[FNRML_PROP];
	float fside_prop = dataref_val[FSIDE_PROP];
	float faxil_prop = dataref_val[FAXIL_PROP];
	float fnrml_aero = dataref_val[FNRML_AERO];
	float fside_aero = dataref_val[FSIDE_AERO];
	float faxil_aero = dataref_val[FAXIL_AERO];
	float fnrml_gear = dataref_val[FNRML_GEAR];
	float fside_gear = dataref_val[FSIDE_GEAR];
	float faxil_gear = dataref_val[FAXIL_GEAR];
	float m_total = dataref_val[M_TOTAL];
	float the = dataref_val[THETA];
	float psi = dataref_val[PSI];
	float phi = dataref_val[PHI];

	float ratio = MPD_fltlim(groundspeed*0.2,0.0,1.0);
	float a_nrml= MPD_fallout(fnrml_prop+fnrml_aero+fnrml_gear,-0.1,0.1)/MPD_fltmax2(m_total,1.0);
	float a_side= (fside_prop+fside_aero+fside_gear)/MPD_fltmax2(m_total,1.0)*ratio;
	float a_axil= (faxil_prop+faxil_aero+faxil_gear)/MPD_fltmax2(m_total,1.0)*ratio;

//	printf("%f %f %f %f %f %f\n", the, psi, phi, a_side, a_nrml, a_axil);
	state->theta=the;
	state->psi=psi;
	state->phi=phi;
	state->a_side=a_side;
	state->a_nrml=a_nrml;
	state->a_axil=a_axil;
}

int xplane_udp_setup(xplane_udp_state_t *state) {
	int s_bcn;
	struct sockaddr_in si_bcn, si;
	struct ip_mreq mreq;
	int slen_bcn = sizeof(si_bcn);
	int recv_len;
	int yes=1;
	char buf[BUFLEN];
	int slen=sizeof(struct sockaddr_in);
	
#ifdef BECN
	struct becn_struct {
		unsigned char beacon_major_version;
		unsigned char beacon_minor_version;
		int application_host_id;
		int version_number;
		unsigned int role;
		unsigned short port;
		xchr computer_name[500];
	} bcn;

	// multicast socket	
	s_bcn=socket(AF_INET, SOCK_DGRAM, 0);
	if (s_bcn < 0) {
		printf("coudn't create beacon socket\n");
		exit(1);
	}
	memset((char *)&si_bcn, 0, sizeof(s_bcn));
	si_bcn.sin_family = AF_INET;
	si_bcn.sin_addr.s_addr = htonl(INADDR_ANY);
	si_bcn.sin_port = htons(MULTICAST_PORT);

	mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	setsockopt(s_bcn, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

	setsockopt(s_bcn, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));


	if (bind(s_bcn, (struct sockaddr *) &si_bcn, slen_bcn)<0) {
		printf("couldn't bind beacon socket\n");
		exit(1);
	}
	while(1) {
		int len = recvfrom(s_bcn, buf, sizeof(buf), 0, (struct sockaddr *) &si_bcn, &slen_bcn);
		printf("%d\n", len);
		memcpy(&bcn, buf, sizeof(bcn));
		
		printf("major=%d minor=%d name=%s\n", bcn.beacon_major_version, bcn.beacon_minor_version, bcn.computer_name);
	}
#endif
	// socket for rrefs
	state->s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset((char*) &si, 0, sizeof(si));
	si.sin_family = AF_INET;
	si.sin_port = htons(PORT);
	inet_aton(state->server, &(si.sin_addr));

	int i;
	for (i=0; i<15; i++) {
		//printf("%s %d\n", dataref_name[i], i);
		register_dataref(state->s, &si, i, dataref_name[i], state->sample); 
	}
}

void *xplane_udp_run(void *state_v) {
	xplane_udp_state_t *state = (xplane_udp_state_t *) state_v;
	int recv_len;
	int slen;
	char buf[BUFLEN];
	struct sockaddr_in si;

	memset((char*) &si, 0, sizeof(si));
	si.sin_family = AF_INET;
//	si.sin_port = htons(PORT);
	inet_aton(state->server, &(si.sin_addr));

	while(state->run) {
		struct dref_struct_out *dref;
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
			//printf("got data\n");
			recv_len = recvfrom(state->s, buf, BUFLEN, 0, (struct sockaddr *) &si, &slen);
			//printf("received %d\n", recv_len);
			dref = (struct dref_struct_out *) (buf+5);
			while (cnt + sizeof(struct dref_struct_out) <= recv_len) {
				//printf("%d\t%f\n", dref->dref_en, dref->dref_flt);
				if (dref->dref_en>=0 && dref->dref_en<14) {
					dataref_val[dref->dref_en] = dref->dref_flt;
				}
				if (dref->dref_en == PAUSE) {
					paused = (int) dref->dref_flt;
				}
				cnt += sizeof(struct dref_struct_out);
				dref++;
			}
			calculate_accel(state);
		} else {
			//printf("timeout...\n");
			xplane_udp_setup(state);
			paused = 1;
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

