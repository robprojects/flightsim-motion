#include <math.h>

#define N 6

#include <GL/glut.h>

#include <stdio.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include <getopt.h>
#include <float.h>

#include "xplane.h"
#include "nolimits2.h"
#include "motion.h"
#include "washout.h"
#include "geo6dof.h"
#include "actuator.h"

#include "sys/time.h"

#define NUM_GRAPH 3

int graph_pos = 0;
float graph[NUM_GRAPH][640];

struct motion_geo geo;
struct motion_pos pos;
struct motion_state st;

int stopped = 1;
int stop_height = 0;

int hilight = 0;

//int t = 0;
struct sim_params params;
struct compute_state state;

extern actuator_state act_state[6];

int (*source_readnext)(void *sstate, double *Faa, double *Oaa, int *valid);
void *source_state;

int do_washout = 0;
int nodevice = 0;

void display(void);

void init(void)
{
	glClearColor(0.0f,0.0f,0.0f,1.0f);
  	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	/*glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LESS);
	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);*/
}

GLfloat aspect;

void reshape(GLsizei width, GLsizei height) {
	if (height == 0) height =1;
	aspect = (GLfloat) width / (GLfloat)height;
	glViewport(0,0,width, height);
/*	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, aspect, 0.1f, 100.0f);*/
}

double Oaa_manual[3] = {0.0, 0.0, 0.0};

void processKeys(unsigned char key, int x, int y) {
	float fraction = 0.0025f;
	if (!do_washout) {	
		switch(key) {
			case 'd': pos.theta += fraction; break;
			case 'a': pos.theta -= fraction; break;
			case 'w': pos.phi += fraction; break;
			case 's': pos.phi -= fraction; break;
			case 'x': pos.psi += fraction; break;
			case 'z': pos.psi -= fraction; break;
			case 'r':
				pos.psi = pos.theta = pos.phi = 0.0;
				pos.T[0] = pos.T[1] = 0.0;
				pos.T[2] = geo.mid_height;
				break;
		}
	} else {
		float fraction = 10.0f;
		switch(key) {
			case 'd': Oaa_manual[0] += fraction; break;
			case 'a': Oaa_manual[0] -= fraction; break;
 			case 'w': Oaa_manual[1] += fraction; break;
			case 's': Oaa_manual[1] -= fraction; break;
			case 'x': Oaa_manual[2] += fraction; break;
			case 'z': Oaa_manual[2] -= fraction; break;
		}
	}
	switch(key) {
                case 'o': hilight = (hilight+1)%8;
		break;
		case 't': stopped = !stopped; 
		break;
	}
	glutPostRedisplay();
	//print_geo();
}

void processSpecialKeys(int key, int xx, int yy) {

	float fraction = 0.0025f;

	switch (key) {
		case GLUT_KEY_LEFT :
			pos.T[0]+=fraction;
			break;
		case GLUT_KEY_RIGHT :
			pos.T[0]-=fraction;
			break;
		case GLUT_KEY_UP :
			pos.T[1]+=fraction;
			break;
		case GLUT_KEY_DOWN :
			pos.T[1]-=fraction;
			break;
		case GLUT_KEY_PAGE_UP :
			pos.T[2]+=fraction;
			break;
		case GLUT_KEY_PAGE_DOWN :
			pos.T[2]-=fraction;
			break;
	}
	glutPostRedisplay();
	//print_geo();
}

double Faa_manual[3] = {0.0, 0.0, 0.0};

int manual_readnext(void *sstate, double *Faa, double *Oaa, int *valid) {

	Faa[0] = Faa_manual[0];
	Faa[1] = Faa_manual[1];
	Faa[2] = Faa_manual[2];

	Oaa[0] = Oaa_manual[0];
	Oaa[1] = Oaa_manual[1];
	Oaa[2] = Oaa_manual[2];
	
	*valid = 1;
	return 0;
}

void processSpecialKeysM(int key, int xx, int yy) {

	float fraction = 1.0f;

	switch (key) {
		case GLUT_KEY_LEFT :
			Faa_manual[0] += fraction;
			break;
		case GLUT_KEY_RIGHT :
			Faa_manual[0] -= fraction;
			break;
		case GLUT_KEY_UP :
			Faa_manual[1] += fraction;
			break;
		case GLUT_KEY_DOWN :
			Faa_manual[1] -= fraction;
			break;
		case GLUT_KEY_PAGE_UP :
			Faa_manual[2] += fraction;
			break;
		case GLUT_KEY_PAGE_DOWN :
			Faa_manual[2] -= fraction;
			break;
	}
}


void init_washout(int steps) {
	double Faa[3], Oaa[3];
	int i, valid;
	if (!do_washout) return;
	
	(*source_readnext)(source_state, Faa, Oaa, &valid);
	for (i=0; i<steps; i++) {
		compute2(Faa, Oaa, &state, &params, &pos);
	}
}

double Faa[3]; // three body axis accelerations
double Oaa[3]; // three angular rates (yaw, pitch, roll)

void timer(int v) {
	int valid;

	int done = (*source_readnext)(source_state, Faa, Oaa, &valid);

	stopped = !valid;

	// washout filter
	if (do_washout) {
		compute2(Faa, Oaa, &state, &params, &pos);
		pos.T[2] += geo.mid_height;
	}

	if (!done) glutTimerFunc((int) 1000.0/SAMPLE, timer, 0);

	// update 6dof
	struct motion_pos *pos_out;
	struct motion_pos stopped_pos;
	stopped_pos.psi = 0.0;
	stopped_pos.theta = 0.0;
	stopped_pos.phi = 0.0;
	stopped_pos.T[0] = 0.0;
	stopped_pos.T[1] = 0.0;
	stopped_pos.T[2] = geo.min_height + ((geo.mid_height-geo.min_height)/500.0)*((float)stop_height);

	if (!stopped) {
		if (stop_height < 500) {
			stop_height+=100/SAMPLE;
			pos_out = &stopped_pos;
		} else {
			pos_out = &pos;
		}
	} else {
		if (stop_height > 0) stop_height-=100/SAMPLE;
		pos_out = &stopped_pos;
	}

	inverse_kinematics(&geo, pos_out, &st);



	int len[N];
	int i;
	for (i=0; i<N; i++) {
		len[i] = (int)((1.0-((st.length[i]-geo.act_min)/geo.act_range))*65535.0);
		if (len[i] <= 500) len[i] = 500;
		if (len[i] >= 65535) len[i] = 65535;

		//printf("act %d\tlen=%f\tval=%d\n", i, st.length[i]*100.0, len[i]); 
	}

	if (!nodevice) send_serial_command(len);

	switch (hilight) {

		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		graph[0][graph_pos] = (float) len[hilight];
		graph[1][graph_pos] = (float) act_state[hilight].actual_pos;
		graph[2][graph_pos] = (float) act_state[hilight].actual_pos;
		break;
		case 6:
		graph[0][graph_pos] = pos.theta;
		graph[1][graph_pos] = pos.psi;
		graph[2][graph_pos] = pos.phi;
		break;
		case 7:
		graph[0][graph_pos] = pos.T[0];
		graph[1][graph_pos] = pos.T[1];
		graph[2][graph_pos] = pos.T[2];
		break;
	}
	graph_pos = (graph_pos + 1)%640;

	glutPostRedisplay();
}

#define NOTEST

int null_readnext(void *sstate, double *Faa, double *Oaa, int *valid) {
	Oaa[0] = Oaa[1] = Oaa[2] = 0.0;
	Faa[0] = Faa[1] = Faa[2] = 0.0;
	*valid = 1;
	
	return 0;
}

void *file_init(char *file) {
	FILE *f = fopen(file, "r");

	if (!f) {
		printf("Error: could not open file %s\n", file);
		exit(1);
	}

	return (void *) f;
}

int file_readnext(void *sstate, double *Faa, double *Oaa) {
	float theta, psi, phi, y, z, x;
	FILE *f=(FILE *) sstate;
	fscanf(f, "%f %f %f %f %f %f\n", &theta, &psi, &phi, &y, &z, &x);
	Oaa[0] = theta; Oaa[1] = psi; Oaa[2] = phi;
	Faa[0] = x; Faa[1] = y; Faa[2] = z;
	
	return feof(f);
}

void file_close(void *sstate) {
	fclose((FILE *)sstate);
}

void print_usage() {
	printf("Usage:\n");
	printf("motion -x <server> to get data from X-Plane UDP\n");
	printf("motion -f <file> to get data from file\n");
}

int main(int argc, char **argv) {
	memset(&state, sizeof(struct compute_state), 0);		
	sim_params_init(&params);

	params.final_filt = 0;



	int i;


	// see if we're starting high...
	printf("pos = %d\n", act_state[0].actual_pos);
	if (act_state[0].actual_pos < 40000) stop_height = 499;

	glutInit(&argc, argv);

	static struct option long_options[] = 
	{
		{"file", required_argument, 0, 'f'},
		{"xplane", required_argument, 0, 'x'},
		{"nolimits2", required_argument, 0, 'l'},
		{"manual", no_argument, 0, 'm'},
		{"nodevice", no_argument, 0, 'n'},
		{0,0,0,0}
	};
	int opt;
	int longindex=-1;

	source_readnext = null_readnext;

	while ((opt = getopt_long(argc, argv, "nmf:x:l:", long_options, &longindex)) != -1) {
		switch(opt) {
			case 'n':
			nodevice = 1;
			break;
			case 'f':
			source_state = file_init(optarg);
			source_readnext = file_readnext;
			do_washout = 1;
			break;
			case 'm':
			source_readnext = manual_readnext;
			do_washout = 1;
			break;
			case 'x':
			source_state = xplane_init(optarg, (int)SAMPLE);
			source_readnext = xplane_readnext;
			do_washout = 1;
			params.final_filt = 1;
			break;
			case 'l':
			source_state = nolimits2_init(optarg, (int)SAMPLE);
			source_readnext = nolimits2_readnext;
			do_washout = 1;
			break;
			default: print_usage();
				exit(-1);
		}
	}

	if (!nodevice) setup_serial();

	glutInitDisplayMode(GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow("simulator");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	if (!do_washout) {
		glutKeyboardFunc(processKeys);
		glutSpecialFunc(processSpecialKeys);
	} else {
		glutKeyboardFunc(processKeys);
		glutSpecialFunc(processSpecialKeysM);
	}
	glutTimerFunc(100, timer, 0);
	init();

	init_geometry(&geo, 0.747, 0.5623, 0.85163, 0.70986, 0.2835, 0.316, 0.178);


	pos.psi = 0.0;
	pos.theta = 0.0;
	pos.phi = 0.0;
	pos.T[0] = 0.0;
	pos.T[1] = 0.0;
	pos.T[2] = geo.mid_height;

	init_washout(10*SAMPLE);

#ifndef NOTEST
	double pr = 0.5;
	for (i=0; i<10; i++) {
		printf("*********** pr=%.2f ***********\n", pr);
		init_geometry(&geo, 0.8, pr, 0.8, 0.65, 0.3, 0.1);
		test_geometry();
		pr+=0.05;
	}
#endif
	
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
	return 0;
}

void bargraph(int act) {
// bar

	int width=150, height=20, top=(480-act*(height+5));
	int bwidth=150-((int) (act_state[act].actual_pos * 150)/(1<<16));
	int cwidth=150-((int) (act_state[act].cmd_pos * 150)/(1<<16));
	struct timeval now, diff;

	servo_state_t sstate = FLAGS_STATE(act_state[act].last_flags);

	gettimeofday(&now, NULL);

	float r=0.0f,g=0.0f,b=0.0f;

	switch (sstate) {
		case SERVO_INIT:
		case SERVO_FINDHOME:
			b=1.0f;
		break;
		case SERVO_RUN:
			g=1.0f;
		break;
		case SERVO_ERROR:
			r=1.0f;
		break;
		case SERVO_STOP:
		case SERVO_STOP_NOHOME:	
			r=1.0f; g=1.0f;
		break;
	}

	if (act_state[act].active) {
		timersub(&now, &(act_state[act].last_response), &diff);
		float dim = (1000000-diff.tv_usec)/1000000.0;
		r=r*=dim;
		g=g*=dim;
		b=b*=dim;
		if (diff.tv_sec!=0) { r=1.0f; g=0.0f; b=0.0f; }
		glBegin(GL_QUADS);
		glColor3f(r, g, b); // red
		glVertex2f(0, top);
		glVertex2f(0+bwidth, top);
		glVertex2f(0+bwidth, top-height);
		glVertex2f(0, top-height);
		glVertex2f(0, top);
		glEnd();
	}

	// outline
	glBegin(GL_LINES);
	if (act_state[act].active) {
		glColor3f(1.0f,1.0f,1.0f); // white
                if (act_state[act].last_flags & FLAG_BRAKE_LIMIT) {
			glColor3f(1.0f,0.0f,0.0f); // red 
		} else {
			glColor3f(1.0f,1.0f,1.0f); // white
		}
	} else {
		glColor3f(0.5f,0.5f,0.5f); // gray
	}
	glVertex2f(0, top);
	glVertex2f(0+width, top);
	glVertex2f(0+width, top);
	glVertex2f(0+width, top-height);
	glVertex2f(0+width, top-height);
	glVertex2f(0, top-height);
	glVertex2f(0, top-height);
	glVertex2f(0, top);
	glEnd();

	// cmd pos line
	glBegin(GL_LINES);
	glColor3f(1.0f,1.0f,1.0f); // white
	glVertex2f(0+cwidth, top);
	glVertex2f(0+cwidth, top-height);
	glEnd();
}


void display(void) {
	int i;

	glEnable(GL_DEPTH_TEST);
	//gluPerspective(45.0f, aspect, 0.1f, 100.0f);
	//glMatrixMode(GL_PROJECTION);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, 0.5f);
	//glRotatef(10.0f, 1.0f, 0.0f, 1.0f);
	gluLookAt(0.0f, 1.0f, 3.0f,
		 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, aspect, 0.1f, 100.0f);
	//gluLookAt(0.0f,1.5f,5.0f,0.0f,1.3f,4.0f,0.0f,1.0f,0.0f);
  	//glTranslatef(0.0f, 0.0f, -7.0f);
	
	//glMatrixMode(GL_MODELVIEW);


//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
	//glEnable(GL_DEPTH_TEST);

//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(45.0f, aspect, 0.1f, 100.0f);

  	//glTranslatef(0.0f, 0.0f, -7.0f);
	  //glRotatef(80.0f, 1.0f, 0.0f, 0.0f);
 	 //gluLookAt(0.0f, 1.5f/* + z*/, 5.0f,
//		0.0f, 1.3f, 4.0f,
	//	0.0f, 1.0f, 0.0f);

	// base
 	glColor3f(1.0, 0.0, 0.0);
 	glBegin(GL_POLYGON);
	for (i=0; i<6; i++) {
		glVertex3f(geo.base[i][0], geo.base[i][2], geo.base[i][1]);
	}
 	glEnd();

	// actuators
	glLineWidth(3.0);
  	for (i=0; i<N; i++) {
  		glColor3f((i==hilight) ? 0.0 : 1.0, 1.0, 1.0);
    		glBegin(GL_LINES);
    		glVertex3f(geo.b[i][0], geo.b[i][2], geo.b[i][1]);
    		glVertex3f(st.p[i][0], st.p[i][2], st.p[i][1]);
    		glEnd();
  	}

 	// platform
 	glColor3f(0.0, 1.0, 0.0);
 	glBegin(GL_POLYGON);
	for (i=0; i<6; i++) {
 		glVertex3f(st.platform[i][0], st.platform[i][2], st.platform[i][1]);
	}
  	glEnd();

	glMatrixMode(GL_PROJECTION);
	glDisable(GL_DEPTH_TEST);
	glLoadIdentity();
	gluOrtho2D(0, 640, 0, 480);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// graph

	glLineWidth(1.0);

	float graph_min = FLT_MAX;
	float graph_max = -FLT_MAX;
	for (i=0; i<(hilight<6 ? 2 : NUM_GRAPH); i++) {
		int j=0;
		for (j=0; j<640; j++) {
			if (graph[i][j] < graph_min) graph_min = graph[i][j];
			if (graph[i][j] > graph_max) graph_max = graph[i][j];
		}
	}
	//printf("min %f, max %f\n", graph_min, graph_max); 
	float scale = 200.0/((float)(graph_max-graph_min));

	for (i=0; i<NUM_GRAPH; i++) {
		switch(i){
			case 0:
			glColor3f(0.5, 0.0, 0.0); break;
			case 1:
			glColor3f(0.0, 0.5, 0.0); break;
			case 2:
			glColor3f(0.0, 0.0, 0.5); break;
		}
		int j;
		float prev_point = graph[i][graph_pos];
		for (j=1; j<640;j++) {
			float point = graph[i][(j+graph_pos)%640];
			glBegin(GL_LINES);
			glVertex2f(j, (float)(prev_point-graph_min)*scale);
			glVertex2f(j+1, (float)(point-graph_min)*scale);
			glEnd();
			prev_point = point;	
		}
	}

	glLineWidth(3.0);

	// bar graph
	for (i=0; i<6; i++) {
		bargraph(i);
	}

	if (stopped || stop_height < 500) {
		char stoptext[100];
		if (!stopped) {
			sprintf(stoptext, "STARTING IN %d/500", stop_height);
		} else {
			sprintf(stoptext, "STOPPED - 'T' TO TOGGLE %d", hilight);
		}
		glColor3f(1.0, 1.0, 1.0);
		char *c = stoptext;
		int pos = 320 - (strlen(stoptext)*9)/2;
		for ( ; *c!='\0'; c++) {
			glRasterPos2f(pos,240);
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
			pos+=9;
		}
	}
	char gtext[100];
	switch (hilight) {

		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		sprintf(gtext, "ACTUATOR %d", hilight);
		break;
		case 6:
		sprintf(gtext, "pos.theta/psi/phi");
		break;
		case 7:
		sprintf(gtext, "T[3]");
		break;
	}
	glColor3f(1.0, 1.0, 1.0);
	char *c = gtext;
	int pos = 320 - (strlen(gtext)*9)/2;
	for ( ; *c!='\0'; c++) {
		glRasterPos2f(pos,50);
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
		pos+=9;
	}

	
	glFlush();
  	glutSwapBuffers();
}


