#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define HFENABLE 1
#define TCENABLE 1

#include "motion.h"
#include "washout.h"
#include "matrix.h"


double filter(struct f_co *cf, double in, struct f_state *fs) {
	double ret = cf->a1 * in + cf->a2 * fs->in_prev[0] + cf->a3 * fs->in_prev[1] -
		cf->b1 * fs->out_prev[0] - cf->b2 * fs->out_prev[1];

	fs->in_prev[1] = fs->in_prev[0];
	fs->in_prev[0] = in;
	fs->out_prev[1] = fs->out_prev[0];
	fs->out_prev[0] = ret;	
	return ret;
} 

/* 

 Low pass IIR filter (used for tilt coordination)
 H(s) = Wn^2 / ( s^2 + 2*Z*Wn s + Wn^2)   

 Calculate 2nd order IIR filter coefficients by bilinear z-transform

*/
void fc_lopass(double sample_rate, double Wn, double Z, struct f_co *cf) {
	double alpha = Wn*Wn;
	double beta = 2.0*Z*Wn;

	double Wac = tan(Wn/(sample_rate*2.0));

	double norm = 1.0/(Wac*Wac) + beta/Wac + alpha;	

	cf->a1 = alpha/norm;
	cf->a2 = 2*alpha/norm;
	cf->a3 = alpha/norm;
	cf->b1 = (-2/(Wac*Wac)+2*alpha)/norm;
	cf->b2 = (1/(Wac*Wac)-beta/Wac+alpha)/norm;
}

/*
void fc_hipass(double sample_rate, double f, struct f_co *cf) {
	double r = sqrt(2.0);   
   	double c = tan(M_PI * f / sample_rate);

	cf->a1 = 1.0 / ( 1.0 + r * c + c * c);
	cf->a2 = -2*cf->a1;
	cf->a3 = cf->a1;
	cf->b1 = 2.0 * ( c*c - 1.0) * cf->a1;
	cf->b2 = ( 1.0 - r * c + c * c) * cf->a1;

	printf("hp %f %f %f %f %f\n", cf->a1, cf->a2, cf->a3, cf->b1, cf->b2);

}
*/

/* 

 High pass IIR filter
 H(s) = s^2 / ( s^2 + 2*Z*Wn s + Wn^2)   

 Calculate 2nd order IIR filter coefficients by bilinear z-transform

*/
void fc_hipass_1(double sample_rate, double Wn, double Z, struct f_co *cf) {
	double Wac = tan(Wn/(sample_rate*2.0));

	double norm = 1.0+2.0*Z*Wn*Wac+Wn*Wn*Wac*Wac;
	
	cf->a1 = 1.0/norm;
	cf->a2 = -2.0/norm;
	cf->a3 = 1.0/norm;
	cf->b1 = (-2.0+2.0*Wn*Wn*Wac*Wac)/norm;
	cf->b2 = (1.0-2.0*Z*Wn*Wac+Wn*Wn*Wac*Wac)/norm;

}

/* 

 High pass IIR filter
 H(s) = s / s + Wb   

 Calculate 1st order IIR filter coefficients by bilinear z-transform

*/
void fc_hipass_2(double sample_rate, double Wb, struct f_co *cf) {
	double Wac = tan(Wb/(sample_rate*2.0));

	double norm = 1.0+Wb*Wac;
	
	cf->a1 = 1.0/norm;
	cf->a2 = -1.0/norm;
	cf->a3 = 0.0;
	cf->b1 = (-1.0+Wb*Wac)/norm;
	cf->b2 = 0.0;

}

int scale_and_limit(double *in, double *out, struct sim_params *params, sl_t sl) {
        double *scale, *limit;
	switch (sl) {
		case F_LP:
			scale = params->Faa_scale_lp;
			limit = params->Faa_limit_lp;
			break;
		case F_O:
			scale = params->Oaa_scale_hp;
			limit = params->Oaa_limit_hp;
			break;
		case F_HP:
			scale = params->Faa_scale_hp;
			limit = params->Faa_limit_hp;
			break;
	}

	// limit
	int i;
	for (i=0; i<3; i++) {
		out[i] = in[i];
		if (in[i]>limit[i]) out[i] = limit[i];
		if (in[i]<-limit[i]) out[i] = -limit[i];
	}
	// scale
	for (i=0; i<3; i++) out[i]*= scale[i];
}

int lp_filter_faa(double *in, double *out, struct sim_params *params, struct f_state *fs) {
	int i;

	for (i=0; i<3; i++) out[i] = filter(&(params->lpfilt_faa[i]), in[i], &fs[i]);
} 

int lp_filter_faa_final(double *in, double *out, struct sim_params *params, struct f_state *fs) {
	int i;

	for (i=0; i<3; i++) out[i] = filter(&(params->lpfilt_faa_final[i]), in[i], &fs[i]);
} 

int lp_filter_oaa_final(double *in, double *out, struct sim_params *params, struct f_state *fs) {
	int i;

	for (i=0; i<3; i++) out[i] = filter(&(params->lpfilt_oaa_final[i]), in[i], &fs[i]);
}
 
int tilt_coord_rl(double *in, struct sim_params *params, double *tc_pitch, double *tc_roll) {
	double roll_tgt, pitch_tgt;

	// side to roll
	roll_tgt = atan(in[1]/9.8);
	if ((roll_tgt - *tc_roll) > params->max_roll) *tc_roll = *tc_roll + params->max_roll;
	if ((roll_tgt - *tc_roll) < -params->max_roll) *tc_roll = *tc_roll - params->max_roll;

	// accel to pitch
	pitch_tgt = atan(in[0]/9.8 * cos(*tc_roll));

	if ((pitch_tgt - *tc_pitch) > params->max_pitch) *tc_pitch = *tc_pitch + params->max_pitch;
	if ((pitch_tgt - *tc_pitch) < -params->max_pitch) *tc_pitch = *tc_pitch - params->max_pitch;

	//printf("roll tgt %f, pitch tgt %f\n", *roll_tgt, *pitch_tgt);
}

int hp_filter_faa(double *in, double *out, struct sim_params *params, struct f_state *fs) {
	// HP filter
	// first biquad cascade
	double Faa_hp[3];
	int i;
	for (i=0; i<3; i++) Faa_hp[i] = filter(&(params->hpfilt_faa[i]), in[i], &fs[i*2]);
	// second cascade
	for (i=0; i<3; i++) out[i] = filter(&(params->hpfilt_faa_c[i]), Faa_hp[i], &fs[i*2+1]);
}

int hp_filter_oaa(double *in, double *out, struct sim_params *params, struct f_state *fs) {
	int i;
	for (i=0; i<3; i++) out[i] = filter(&(params->hpfilt_oaa[i]), in[i], &fs[i]);
	//printf("hp filt omega: %f %f %f\n", out[0], out[1], out[2]);

}


int faa_rot(double *in, double *out, struct motion_pos *pos) {
	// rotate by current tilt angle 
	double RB[9];
	int i;
	//printf("pre-rot: %f %f %f\n", in[0], in[1], in[2]);

	rot_matrix(RB, pos->psi, pos->theta, pos->phi);
	for (i=0; i<3; i++) {
		out[i] = RB[i] * in[0] + RB[i+3] * in[1] + RB[i+6] * in[2];
	}
	//printf("post-rot: %f %f %f\n", out[0], out[1], out[2]);
}

int oaa_rot(double *in, double *out, struct motion_pos *pos) {
	double RB[9];
	double psi = pos->psi;
	double theta = pos->theta;
	double phi = pos->phi;

	double tant = sin(theta)/cos(theta);

	int i;

	RB[0] = 1.0;
	RB[1] = sin(phi)*tant;
	RB[2] = cos(phi)*tant;

	RB[3] = 0.0;
	RB[4] = cos(phi);
	RB[5] = -sin(phi);

	RB[6] = 0.0;
	RB[7] = sin(phi)/cos(theta);
	RB[8] = cos(phi)/cos(theta);
 
	for (i=0; i<3; i++) {
		out[i] = RB[i] * in[0] + RB[i+3] * in[1] + RB[i+6] * in[2];
	}
}

int sub_g(double *in, double *out, struct motion_pos *pos) {
	// g
	out[0] = in[0] - 9.8*sin(pos->theta);
	out[1] = in[1] + 9.8*cos(pos->theta)*sin(pos->phi);
	out[2] = in[2] + 9.8*cos(pos->theta)*cos(pos->phi); 
}

int integrate2x(double *in, double *out, double *sum1, double *sum2) {
	int i;
	for (i=0; i<3; i++) {
		sum1[i] += in[i] * (1.0/SAMPLE); // FIXME multiply by sampling interval
		sum2[i] += sum1[i] * (1.0/SAMPLE); // FIXME multiply by sampling interval
		out[i] = sum2[i];
	}
}

int integrate(double *in, double *out, double *sum1) {
	int i;
	// integrate once 
	for (i=0; i<3; i++) {
		sum1[i] += in[i] * (1.0/SAMPLE); // FIXME multiply by sampling interval
		out[i] = sum1[i];
	}
	//printf ("hp filt omega, int : %f %f %f\n", Oaa_dot[0], Oaa_dot[1], Oaa_dot[2]);
}

int deg2rad(double *in, double *out) {
	int i;
	//printf("omega deg: %f %f %f\n", in[0], in[1], in[2]);

	for (i=0; i<3; i++) out[i] = (in[i]/360.0) * (2 * M_PI); 
}

int differentiate(double *in, double *out, double *last) {
	int i;
	//printf("omega: %f %f %f\n", in[0], in[1], in[2]);
	// FIXME: Rads/sec (10 is sample rate)
	for (i=0; i<3; i++) { out[i] = (in[i] - last[i])*SAMPLE; last[i] = in[i]; }
	//printf("omega_v: %f %f %f\n", out[0], out[1], out[2]);
}

int sim_params_init(struct sim_params *params) {
	int i;
	
	params->Faa_scale_hp[0] = 0.8;
	params->Faa_scale_hp[1] = 0.8;
	params->Faa_scale_hp[2] = 0.8;

	params->Faa_limit_hp[0] = 5.0;
	params->Faa_limit_hp[1] = 5.0;
	params->Faa_limit_hp[2] = 5.0;

	params->Oaa_scale_hp[0] = 0.8;
	params->Oaa_scale_hp[1] = 0.8;
	params->Oaa_scale_hp[2] = 0.8;

	params->Oaa_limit_hp[0] = 100.0;
	params->Oaa_limit_hp[1] = 100.0;
	params->Oaa_limit_hp[2] = 100.0;

	params->Faa_scale_lp[0] = 0.8;
	params->Faa_scale_lp[1] = 0.8;
	params->Faa_scale_lp[2] = 0.8;

	params->Faa_limit_lp[0] = 10.0;
	params->Faa_limit_lp[1] = 10.0;
	params->Faa_limit_lp[2] = 10.0;

	params->max_pitch = 0.005;
	params->max_roll = 0.005;

	params->d_oaa = 2;

	params->lpfilt_faa_z = 0.707;
	params->lpfilt_faa_o = 1.25;

	// compute filter coefficients
	for (i=0; i<3; i++) {
		fc_lopass(SAMPLE, params->lpfilt_faa_o, params->lpfilt_faa_z, &(params->lpfilt_faa[i]));
	}	

	params->lpfilt_faa_final_z = 0.707;
	params->lpfilt_faa_final_o = 4.0;
	for (i=0; i<3; i++) {
		fc_lopass(SAMPLE, params->lpfilt_faa_final_o, params->lpfilt_faa_final_z, &(params->lpfilt_faa_final[i]));
	}	
	params->lpfilt_oaa_final_z = 0.707;
	params->lpfilt_oaa_final_o = 4.0;
	for (i=0; i<3; i++) {
		fc_lopass(SAMPLE, params->lpfilt_oaa_final_o, params->lpfilt_oaa_final_z, &(params->lpfilt_oaa_final[i]));
	}
	params->hpfilt_faa_z = 0.707;
	params->hpfilt_faa_o = 1.25;
	params->hpfilt_faa_c_o = 0.125;
	for (i=0; i<3; i++) {
		fc_hipass_1(SAMPLE, params->hpfilt_faa_o, params->hpfilt_faa_z, &(params->hpfilt_faa[i]));
		fc_hipass_2(SAMPLE, params->hpfilt_faa_c_o, &(params->hpfilt_faa_c[i]));
	}
	params->hpfilt_faa_2_o = 1.25;
	params->hpfilt_faa_2_z = 0.707;
	params->hpfilt_faa_2_c_o = 0.125;
	for (i=0; i<3; i++) {
		fc_hipass_1(SAMPLE, params->hpfilt_faa_2_o, params->hpfilt_faa_2_z, &(params->hpfilt_faa_2[i]));
		fc_hipass_2(SAMPLE, params->hpfilt_faa_2_c_o, &(params->hpfilt_faa_2_c[i]));
	}	
	params->hpfilt_oaa_z = 0.707;
	params->hpfilt_oaa_o = 5.0;
	for (i=0; i<3; i++) {
		fc_hipass_1(SAMPLE, params->hpfilt_oaa_o, params->hpfilt_oaa_z, &(params->hpfilt_oaa[i]));
	}
}

//#define DUMP(X) printf("DEBUG %s\t%f\t%f\t%f\n", #X, X[0], X[1], X[2])

#define DUMP(X)

int compute2(double *faa, double *oaa, struct compute_state *state, struct sim_params *params, struct motion_pos *pos) {
	double faa_lp[3], faa_lp_f[3];

	// Faa -> scale/limit -> LP filter -> Tilt coord -> Rate limit -> Lib (tilt)
	DUMP(faa);
	scale_and_limit(faa, faa_lp, params, F_LP);
	DUMP(faa_lp);
	lp_filter_faa(faa_lp, faa_lp_f, params, state->fs_l);
	DUMP(faa_lp_f);
	tilt_coord_rl(faa_lp_f, params, &state->tc_pitch, &state->tc_roll);
	double faa_hp[3], faa_g[3], faa_f[3], faa_f_rot[3], faa_rot_f[3], faa_rot_f_i[3];

	// Faa -> scale/limit -> (g) -> HP filter -> euler -> HP filter 2 -> integrate x2 -> Si

	scale_and_limit(faa, faa_hp, params, F_HP);
	sub_g(faa_hp, faa_g, pos);
	hp_filter_faa(faa_g, faa_f, params, state->fs_h);
	faa_rot(faa_f, faa_f_rot, pos);
	hp_filter_faa(faa_f_rot, faa_rot_f, params, state->fs_h2);
	integrate2x(faa_rot_f, faa_rot_f_i, state->faa_sum, state->faa_sum2);
	double faa_rot_f_i_filt[3];
	int i;
	if (params->final_filt)
		lp_filter_faa_final(faa_rot_f_i, faa_rot_f_i_filt, params, state->fs_l_f);
	else
		for (i=0; i<3; i++) faa_rot_f_i_filt[i]=faa_rot_f_i[i];


	DUMP(faa_hp);
	DUMP(faa_g);
	DUMP(faa_f);
	DUMP(faa_f_rot);
	DUMP(faa_rot_f);
	DUMP(faa_rot_f_i);

	#if HFENABLE
	pos->T[0] = faa_rot_f_i[0];//_filt[0];
	pos->T[1] = faa_rot_f_i[1];//_filt[1];
	pos->T[2] = faa_rot_f_i[2];//_filt[2];
	#else
	pos->T[0] = pos->T[1] = 0.0;
	pos->T[2] = 0.0;
	#endif

	// Oaa -> scale/limit -> HP filter -> Tb -> HP filter -> integrate -> add tilt coord
	// convert to radians
	double oaa_r[3], oaa_r_d[3], oaa_r_d2[3], oaa_l[3], oaa_l_f[3], oaa_l_f_r[3], oaa_l_f_r_f[3], oaa_ig[3], oaa_ig2[3];
 
	deg2rad(oaa, oaa_r);
	// differentiate to get angular rate 
	differentiate(oaa_r, oaa_r_d, state->oaa_last);	
        // differentiate again to get angular accel
	differentiate(oaa_r_d, oaa_r_d2, state->oaa_last2);

	// FIXME! Rfactor gives angular accel in radians/s2
	switch(params->d_oaa) {
		case 0:
		for (i=0; i<3; i++) oaa_r_d2[i] = oaa[i];
		break;
		case 1:
		for (i=0; i<3; i++) oaa_r_d2[i] = oaa_r_d[i];
		break;
		case 2:
		break;
	}
	// high pass scale and limit Oaa
	scale_and_limit(oaa_r_d2, oaa_l, params, F_O);
	// HP filter
	hp_filter_oaa(oaa_l, oaa_l_f, params, state->fs_o);
	// Tb

	oaa_rot(oaa_l_f, oaa_l_f_r, pos);

	// HP filter
	hp_filter_oaa(oaa_l_f_r, oaa_l_f_r_f, params, state->fs_o2);

	// integrateW
	integrate(oaa_l_f_r_f, oaa_ig, state->oaa_sum);

	integrate(oaa_ig, oaa_ig2, state->oaa_sum2);

	double oaa_ig_filt[3];

	if (params->final_filt)
		lp_filter_oaa_final(oaa_ig2, oaa_ig_filt, params, state->fs_o_f);
	else
		for (i=0; i<3; i++) oaa_ig_filt[i]=oaa_ig2[i];

	DUMP(state->oaa_sum);
	DUMP(oaa);
	DUMP(oaa_r);
	DUMP(oaa_r_d);
	DUMP(oaa_l);
	DUMP(oaa_l_f);
	DUMP(oaa_ig);

#if TCENABLE	
	pos->theta = state->tc_pitch;
	pos->phi = state->tc_roll; 
	pos->psi = 0.0;
#else
	pos->psi = pos->phi = pos->theta = 0.0;
#endif

#if HFENABLE
	pos->theta += oaa_ig_filt[0];
	pos->psi = oaa_ig_filt[1];
	pos->phi += oaa_ig_filt[2];
#endif

}


