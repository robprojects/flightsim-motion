
struct f_co {
	double a1,a2,a3,b1,b2;
};

struct f_state {
	double in_prev[2];
	double out_prev[2];
};

struct sim_params {
	double Faa_scale_hp[3];
	double Faa_limit_hp[3];
	double Oaa_scale_hp[3];
	double Oaa_limit_hp[3];
	double Faa_scale_lp[3];
	double Faa_limit_lp[3];
	double max_pitch;
	double max_roll;

	int final_filt;
	int d_oaa;
	
	// filter params
	double lpfilt_faa_z;
	double lpfilt_faa_o;
	double lpfilt_faa_final_z;
	double lpfilt_faa_final_o;
	double lpfilt_oaa_final_z;
	double lpfilt_oaa_final_o;
	double hpfilt_faa_z;
	double hpfilt_faa_o;
	double hpfilt_faa_c_o;
	double hpfilt_faa_2_o;
	double hpfilt_faa_2_z;
	double hpfilt_faa_2_c_o;
	double hpfilt_oaa_z;
	double hpfilt_oaa_o;

	// filter coefficients
	struct f_co lpfilt_faa[3];
	struct f_co lpfilt_faa_final[3];
	struct f_co lpfilt_oaa_final[3];
	struct f_co hpfilt_faa[3];
	struct f_co hpfilt_oaa[3];
	struct f_co hpfilt_faa_c[3];
	struct f_co hpfilt_faa_2[3];
	struct f_co hpfilt_faa_2_c[3];
};

struct compute_state {
	struct f_state fs_l[3];
	struct f_state fs_l_f[3];
	struct f_state fs_h[6];
	struct f_state fs_h2[6];
	struct f_state fs_o_f[3];
	double faa_sum[3], faa_sum2[3];
	double oaa_last[3];
        double oaa_last2[3];
	struct f_state fs_o[3];
	struct f_state fs_o2[3];
	double oaa_sum[3];
	double oaa_sum2[3];
	double tc_roll;
	double tc_pitch;
};

typedef enum { F_HP, F_LP, F_O } sl_t;

int sim_params_init(struct sim_params *params);
int compute2(double *faa, double *oaa, struct compute_state *state, struct sim_params *params, struct motion_pos *pos);

