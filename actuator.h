
typedef struct {
	int cmd_pos;
	int actual_pos;
	struct timeval last_response;
	int last_flags;
	int num_responses;
	int active;
} actuator_state;


void setup_serial();
void send_serial_command(int *len); 

#define FLAG_SETCONSTANT (1<<5)
#define FLAG_PONG (1<<6)
#define FLAGS_STATE(X) ((X)&0xf)
#define FLAG_BRAKE_LIMIT (1<<7)

typedef enum { SERVO_INIT=0, SERVO_FINDHOME=1, SERVO_RUN=2, SERVO_ERROR=3, SERVO_STOP=4, SERVO_STOP_NOHOME=5 } servo_state_t;

