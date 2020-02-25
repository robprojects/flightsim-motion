#include <math.h>
#include <stdio.h>

#include "motion.h"
#include "matrix.h"
#include "geo6dof.h"

#if 0
struct {
		char name[16];
		double *val;
		double start, min, max;	
	} param[6] = {
		{"x", &pos.T[0], 0.0, 0.0, 0.0},
		{"y", &pos.T[1], 0.0, 0.0, 0.0},
		{"z", &pos.T[2], 0.0, 0.0, 0.0},
		{"phi", &pos.phi, 0.0, 0.0, 0.0},
		{"theta", &pos.theta, 0.0, 0.0, 0.0},
		{"psi", &pos.psi, 0.0, 0.0, 0.0},
	};

#define RADC(A,B) (A>2?((B)/M_PI*180.0):(B))

void print_geo() {
	int pt,i;
	printf ("PARAM\tMIN\tMAX\tVAL\n");
	for (pt=0; pt<6; pt++) {
		printf("%s\t%.2f\t%.2f\t%.2f\n", param[pt].name, RADC(pt,param[pt].min), RADC(pt,param[pt].max), RADC(pt,*param[pt].val));
	}
	printf("LEG\tBALL\tCARDAN\tLEN\n");
	for (i=0; i<6; i++) {
		printf("%d\t%.2f\t%.2f\t%.2f %c\n", i, RADC(3,st.ball[i]), RADC(3,st.cardan[i]),
			st.length[i],(st.length[i] < geo.act_min ||
			st.length[i] > (geo.act_min + geo.act_range))?'*': ' ');

	}
}

int test_geometry() {
	int i,pt;

	double cmax_f, bmax_f;

	param[2].start = geo.mid_height;
	for (i=0; i<6; i++)
		param[i].min = param[i].max = 0.0;

	for (pt=0; pt<6; pt++) {
		for (i=0; i<2; i++) {
			int j;
			// set all params to start value
			for (j=0; j<6; j++) {
				*(param[j].val) = param[j].start;
//				param[j].min = param[j].max = 0.0;
			}

			int limit = 0;
			while (!limit) {
				inverse_kinematics(&geo, &pos, &st);
				for (j=0; j<6; j++) {
					if (st.length[j] < geo.act_min ||
						st.length[j] > (geo.act_min + geo.act_range)) {
						// overextended
						limit = 1;
					}
				}
				// increment param of interest
				*(param[pt].val) += (i==0) ? 0.01 : -0.01;
			}
			if (i==0) param[pt].max = *(param[pt].val);
			if (i==1) param[pt].min = *(param[pt].val);
		}
	}

	print_geo();
	
	// monte carlo testing
	// generate random value for all parameters

	double ball_max[6], cardan_max[6];
	double ball_min[6], cardan_min[6];
	for (i=0; i<6; i++) {
		ball_max[i]=cardan_max[i]=0.0;
		ball_min[i]=cardan_min[i]=1000.0;
	}
	int k;
	for (k=0; k<1; k++) {
	int overext_cnt=0;

	for (i=0; i<100000; i++) {
		for (pt=0; pt<6; pt++) {
			*(param[pt].val) = ((double)rand()/RAND_MAX*(param[pt].max-param[pt].min))+param[pt].min;
		}
		inverse_kinematics(&geo, &pos, &st);
		int limit=0;
		int j;
		for (j=0; j<6; j++) {
			if (st.length[j] < geo.act_min ||
				st.length[j] > (geo.act_min + geo.act_range)) {
				// overextended
				limit = 1;
			}
		}
		if (limit) overext_cnt++;
		else {
			// valid position, actuators could drive here
			for (j=0; j<6; j++) {
				if (st.ball[j] > ball_max[j]) ball_max[j]=st.ball[j];
				if (st.ball[j] < ball_min[j]) ball_min[j]=st.ball[j];
				if (st.cardan[j] > cardan_max[j]) cardan_max[j]=st.cardan[j];
				if (st.cardan[j] < cardan_min[j]) cardan_min[j]=st.cardan[j];
			}
		}
	}
	printf("overextend = %d\n", overext_cnt);
	printf ("LEG\tMINB\tMAXB\tMINC\tMAXC\n");
	double bmax=-1000, bmin=1000, cmax=-1000, cmin=1000;
	for (i=0; i<6; i++) {
		printf("%d\t%.2f\t%.2f\t%.2f\t%.2f\n", i, ball_min[i]/M_PI*180.0, ball_max[i]/M_PI*180.0, cardan_min[i]/M_PI*180.0, cardan_max[i]/M_PI*180.0);
		if (ball_min[i] < bmin) bmin = ball_min[i];
		if (ball_max[i] > bmax) bmax = ball_max[i];
		if (cardan_min[i] < cmin) cmin = cardan_min[i];
		if (cardan_max[i] > cmax) cmax = cardan_max[i];
	}
	printf("%.2f %.2f %.2f %.2f\n", bmax, bmin, cmax, cmin);
	cmax_f = (fabs(cmin) > fabs(cmax)) ? fabs(cmin) : fabs(cmax);
	bmin = fabs(M_PI/2.0-bmin);
	bmax = fabs(M_PI/2.0-bmax);
	bmax_f = bmin>bmax ? bmin : bmax;
	printf("Maximum cardan angle %.2f\n", cmax_f/M_PI*180.0);
	printf("Maximum ball joint angle (from 90deg) %.2f\n", bmax_f/M_PI*180.0);

	}

	return (cmax_f/M_PI*180.0 < 60.0 &&
		bmax_f/M_PI*180.0 < 25);
}

void random_pos() {

	int pt, j;
	int limit=1;
	while (limit) {
		limit =0;
		for (pt=0; pt<6; pt++) {
			*(param[pt].val) = ((double)rand()/RAND_MAX*(param[pt].max-param[pt].min))+param[pt].min;
		}
		inverse_kinematics(&geo, &pos, &st);
		int j;
		for (j=0; j<6; j++) {
			if (st.length[j] < geo.act_min ||
				st.length[j] > (geo.act_min + geo.act_range)) {
				// overextended
				limit = 1;
			}
		}
	}
}
#endif

double find_height(double r_b, double r_p, double len, double theta) {
	double x = r_p * cos(theta) - r_b;
	double y = r_p * sin(theta);
	return sqrt(len*len - y*y - x*x);
}

int init_geometry(struct motion_geo *geo, double radius_base, double radius_platform, double mid_length, double min_length, double range, double sep_angle, double sep_angle_platform) {

	// base and platform geometry
	int i;
	for (i=0; i<6; i++) {
		double angle = (2*M_PI*(((double)(i/2))/3.0));
		angle += M_PI;
		double angle_p = angle;
		if (i%2) angle+=(sep_angle/2.0);
		else angle-=(sep_angle/2.0);

		if (i%2) angle_p+=(sep_angle_platform/2.0);
		else angle_p-=(sep_angle_platform/2.0);


		geo->base[i][0] = radius_base*sin(angle);
		geo->base[i][1] = radius_base*cos(angle);
		geo->base[i][2] = 0.0;
		geo->platform[i][0] = radius_platform*sin(angle_p - M_PI/3);
		geo->platform[i][1] = radius_platform*cos(angle_p - M_PI/3);
		geo->platform[i][2] = 0.0;
	}
/*	
	geo->base[0][0] = 0.11; 
	geo->base[0][1] = -0.595-0.13;

	geo->base[1][0] = -0.11; 
	geo->base[1][1] = -0.595-0.13;

	geo->base[2][0] = -0.71; 
	geo->base[2][1] = 0.395+0.13;

	geo->base[3][0] = -0.58; 
	geo->base[3][1] = 0.595+0.13;

	geo->base[4][0] = 0.58; 
	geo->base[4][1] = 0.595+0.13;

	geo->base[5][0] = 0.71; 
	geo->base[5][1] = 0.395+0.13;
*/


	// anchor points for actuators
	for (i=0; i<6; i++) {
		// x
		int j = i;
		int k = (j + 5)%6;/*
		if (i%2) {
			k=(j+5)%6;
		} else {
			k=j;
		}*/
		geo->p[i][0] = geo->platform[j][0];
		geo->b[i][0] = geo->base[k][0];
		// y
		geo->p[i][1] = geo->platform[j][1];
		geo->b[i][1] = geo->base[k][1];
		// z
		geo->p[i][2] = geo->platform[j][2];
		geo->b[i][2] = geo->base[k][2];
	}
	// calculate height based on mid length of actuator
	double rho = M_PI/3.0 - 2.0*(sep_angle/2.0);
	//double term1=-radius_platform*sin(rho);
	//double term2=radius_base-radius_platform*cos(rho);
	//geo->mid_height = sqrt(mid_length*mid_length-
	//			term1*term1-term2*term2);
	geo->mid_height = find_height(radius_base, radius_platform, mid_length, M_PI/3.0 - sep_angle/2 - sep_angle_platform/2);
	geo->min_height = find_height(radius_base, radius_platform, min_length, M_PI/3.0 - sep_angle/2 - sep_angle_platform/2);

	geo->act_min = min_length;
	geo->act_range = range;
}

int inverse_kinematics(struct motion_geo *geo, struct motion_pos *pos, struct motion_state *st) {
	double RB[9]; // rotation matrix
	int i;

	rot_matrix(RB, pos->psi, pos->theta, pos->phi);

	// qi = T + RB * pi - bi
	// |qi| = length of leg

	for (i=0; i<6; i++) {
		double L[3];
		int j;
		for (j=0; j<3; j++)
			L[j] = pos->T[j] + geo->p[i][0]*RB[j] +
				geo->p[i][1]*RB[j+3] + geo->p[i][2]*RB[j+6] - geo->b[i][j];

		double length = sqrt(L[0]*L[0] + L[1]*L[1] + L[2]*L[2]);
		st->length[i] = length;

		//printf("act %d length %f\n", i, st->length[i]);

		// theta = arccos P.Q/|P||Q|

		// calculate angle of cardan joints at base
		double cja = acos(L[2]/length);
		// calculate angle of ball joints at platform
		
		// normal of plane on ball joint mounting
		double pnorm[3];
		for (j=0; j<3; j++)
			pnorm[j] = geo->p[i][0]*RB[j] +
				geo->p[i][1]*RB[j+3] + geo->p[i][2]*RB[j+6];

		
		double pnl = sqrt(pnorm[0]*pnorm[0] + pnorm[1]*pnorm[1] + pnorm[2]*pnorm[2]);
 
		double bja = acos((pnorm[0]*L[0] + pnorm[1]*L[1] + pnorm[2]*L[2])/(length*pnl));

		st->ball[i] = bja;
		st->cardan[i] = cja; 

		//printf("leg %d cardan joint angle: %.2lf\t ball joint: %.2lf)\n", i, (cja*180.0)/M_PI, 90.0-(bja*180.0)/M_PI);
	}

	// apply rotation/translation matrix to platform and p points
	for (i=0; i<6; i++) { 
		int j;
		for (j=0; j<3; j++) 
			st->platform[i][j] = pos->T[j] +
				geo->platform[i][0]*RB[j] + geo->platform[i][1]*RB[j+3] + 
				geo->platform[i][2]*RB[j+6];
	}
	for (i=0; i<6; i++) {
		int j;
		for (j=0; j<3; j++)
			st->p[i][j] = pos->T[j] + geo->p[i][0]*RB[j] + geo->p[i][1]*RB[j+3] +
				geo->p[i][2]*RB[j+6];
	}

	for (i=0; i<6; i++) {
		double L[3];
		int j;
		for (j=0; j<3; j++)
			L[j] = st->p[i][j] - geo->b[i][j];

		double length = sqrt(L[0]*L[0] + L[1]*L[1] + L[2]*L[2]);
		//printf("%d before=%f after=%f\n", i, st->length[i], length);
		//st->length[i] = length;
	}

	for (i=0; i<6;i++) {
		double L[3];
		int j;
		for (j=0; j<3; j++)
			L[j] = geo->b[i][j] - geo->b[(i+1)%6][j];
		double length = sqrt(L[0]*L[0] + L[1]*L[1] + L[2]*L[2]);
		//printf("%d length of base leg %f\n", length);

	}



	return 0;
}
