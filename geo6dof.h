int init_geometry(struct motion_geo *geo, double radius_base, double radius_platform, double mid_length, double min_length, double range, double sep_angle, double sep_angle_platform);
int inverse_kinematics(struct motion_geo *geo, struct motion_pos *pos, struct motion_state *st);

