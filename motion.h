#define SAMPLE 100.0

struct motion_geo {
	double p[6][3]; // upper anchor point x,y,z relative to platform origin
	double b[6][3]; // lower anchor point x,y,z relative to base origin
	double base[6][3]; // coordinates of simualtor base
	double platform[6][3]; // coordinates of simulator platform
	double mid_height;
	double min_height;
	double act_min;
	double act_range;
};

struct motion_pos {
	double psi; // yaw
	double theta; // pitch
	double phi; // roll
	double T[3]; // translation vector x,y,z
};

struct motion_state {
	double length[6];
	double p[6][3]; // transformed anchor point
	double platform[6][3]; // transformed platform 
	double ball[6];
	double cardan[6];
};

