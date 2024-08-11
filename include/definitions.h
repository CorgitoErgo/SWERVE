// Define motor ports
#define LEFT_UPPER_BEVEL_MOTOR_1 2
#define LEFT_UPPER_BEVEL_MOTOR_2 13
#define LEFT_LOWER_BEVEL_MOTOR_1 1
#define LEFT_LOWER_BEVEL_MOTOR_2 12
#define RIGHT_UPPER_BEVEL_MOTOR_1 9
#define RIGHT_UPPER_BEVEL_MOTOR_2 19
#define RIGHT_LOWER_BEVEL_MOTOR_1 10
#define RIGHT_LOWER_BEVEL_MOTOR_2 20

// Define rotational sensor ports
#define LEFT_ROTATION_SENSOR_PORT 14
#define RIGHT_ROTATION_SENSOR_PORT 18

#define ZERO_VECTOR INFINITY

// Stuff
int leftX, leftY, rightX, rightY;
int left_turn_speed, right_turn_speed;
float other_angle, target_angle;
bool setAngle = false;

// PID control constants for swerve strafing
const float kP = 1.35;
const float kI = 0.0015;
const float kD = 0.75;