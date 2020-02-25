#include <math.h>

void rot_matrix(double *RB, double psi, double theta, double phi) {
	// rotation matrix
	RB[0] = cos(psi) * cos(theta);
	RB[1] = sin(psi) * cos(theta);
	RB[2] = -sin(theta);
	
	RB[3] = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	RB[4] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	RB[5] = cos(theta)*sin(phi);

	RB[6] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	RB[7] = -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	RB[8] = cos(theta)*cos(phi);
}
/*
void t_matrix(double *T, double psi, double theta, double phi) {
	// rotation matrix
	RB[0] = cos(psi) * cos(theta);
	RB[1] = sin(psi) * cos(theta);
	RB[2] = -sin(theta);
	
	RB[3] = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	RB[4] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	RB[5] = cos(theta)*sin(phi);

	RB[6] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	RB[7] = -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	RB[8] = cos(theta)*cos(phi);
}
*/

