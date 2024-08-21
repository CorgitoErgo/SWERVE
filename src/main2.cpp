// #include "main.h"
// #include "pros/motors.hpp"
// #include "pros/adi.hpp"
// #include "pros/rotation.hpp"
// #include <cmath>

// pros::Controller master(pros::E_CONTROLLER_MASTER);
// pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
// pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);

// int translationL = 0;
// int translationR = 0;

// float target_angleL = 0;
// float target_angleR = 0;

// bool leftflip = false;
// bool rightflip = false;

// template <typename T> int sgn(T val) {
//     return (T(0) < val) - (val < T(0));
// }

// void brake(){
//     luA.brake();
//     luB.brake();
//     llA.brake();
//     llB.brake();
//     ruA.brake();
//     ruB.brake();
//     rlA.brake();
//     rlB.brake();
//     pros::delay(2);
// }

// double closestAngle(double a, double b)
// {
//         // get direction
//         double dir = std::fmod(b, 360.0) - std::fmod(a, 360.0);

//         // convert from -360 to 360 to -180 to 180
//         if (abs(dir) > 180.0)
//             dir = -(sgn(dir) * 360.0) + dir;

//         return dir;
// }

// int getNormalizedSensorAngle(pros::Rotation &sensor)
// {
//     float angle = sensor.get_angle() / 100.0; // Convert from centidegrees to degrees

//     if (angle < -180)
//         angle += 360;
//     else if (angle > 180)
//         angle -= 360;

//     return angle;
// }

// double getAngle(int x, int y)
// {
//     double a = std::atan2(std::abs(y), std::abs(x));

//     if (x < 0){
//         if (y < 0)
//             return (M_PI * -1) + a;
//         else if (y > 0)
//             return M_PI - a;
//         else
//             return M_PI;
//     }
//     else if (x > 0){
//         if (y < 0)
//             return a * -1;
//         else if (y > 0)
//             return a;
//         else
//             return 0;
//     }
//     else{
//         if (y < 0)
//             return -M_PI / 2;
//         else if (y > 0)
//             return M_PI / 2;
//         else
//             return 10000; // return illegal value because a zero vector has no direction
//     }
// }

// double wrapAngle(double angle) {
//     if (angle > 180.0)
//         return angle -= 360;
//     else if (angle < -180.0)
//         return angle += 360;
//     else
//         return angle;
// }

// void setAnglePID(float targetL, float targetR){
//     float left_previous_error = 0;
//     float right_previous_error = 0;
//     float left_integral = 0;
//     float right_integral = 0;

//     while(1){
//         float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
//         float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

//         float left_error = wrapAngle(targetL - left_current_angle);
//         float right_error = wrapAngle(targetR - right_current_angle);

//         left_integral += left_error;
//         right_integral += right_error;

//         float left_derivative = left_error - left_previous_error;
//         float right_derivative = right_error - right_previous_error;

//         left_previous_error = left_error;
//         right_previous_error = right_error;

//         float left_motor_speed = lkP * left_error + lkI * left_integral + lkD * left_derivative;
//         float right_motor_speed = rkP * right_error + rkI * right_integral + rkD * right_derivative;

//         luA.move_velocity(-translationL + left_turn_speed);
//         luB.move_velocity(-translationL + left_turn_speed);
//         llA.move_velocity(-translationL - left_turn_speed);
//         llB.move_velocity(-translationL - left_turn_speed);

//         ruA.move_velocity(translationR + right_turn_speed);
//         ruB.move_velocity(translationR + right_turn_speed);
//         rlA.move_velocity(translationR - right_turn_speed);
//         rlB.move_velocity(translationR - right_turn_speed);

//         if(fabs(left_error) <= 2){
//             left_turn_speed = 0;
//         }
//         else{
//             left_turn_speed = left_motor_speed;
//         }
//         if(fabs(right_error) <= 2){
//             right_turn_speed = 0;
//         }
//         else{
//             right_turn_speed = right_motor_speed;
//         }
//         if(fabs(left_error) <= 2 && fabs(right_error) <= 2){
//             left_turn_speed = 0;
//             right_turn_speed = 0;
//             setAngle = false;
//             break;
//         }
//         pros::delay(5);
//     }
// }

// void setDirection(double targetpoint){
//     float left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
//     float right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);

//     double setpointAngleL = closestAngle(left_sensor_angle, targetpoint);
//     double setpointAngleFlippedL = closestAngle(left_sensor_angle, targetpoint + 180.0);

//     double setpointAngleR = closestAngle(right_sensor_angle, targetpoint);
//     double setpointAngleFlippedR = closestAngle(right_sensor_angle, targetpoint + 180.0);

//     if (abs(setpointAngleL) <= abs(setpointAngleFlippedL)){
//         if(translationL < 0){
//             leftflip = false;
//             translationL = -translationL;
//         }
//         if(rightX == 0 && rightY == 0)
//             target_angleL = (left_sensor_angle + setpointAngleL);
//     }
//     else{
//         if(translationL > 0){
//             leftflip = true;    
//             translationL = -translationL;
//         }
//         if(rightX == 0 && rightY == 0)
//             target_angleL = (left_sensor_angle + setpointAngleFlippedL);
//     }

//     if (abs(setpointAngleR) <= abs(setpointAngleFlippedR)){
//         if(translationR < 0){
//             rightflip = false;
//             translationR = -translationR;
//         }
//         if(rightX == 0 && rightY == 0)
//             target_angleR = (right_sensor_angle + setpointAngleR);
//     }
//     else{
//         if(translationR > 0){
//             rightflip = true;
//             translationR = -translationR;
//         }
//         if(rightX == 0 && rightY == 0)
//             target_angleR = (right_sensor_angle + setpointAngleFlippedR);
//     }
//     setAnglePID(target_angleL, target_angleR);
// }

// void initialize(){
//     pros::lcd::initialize();

//     luA.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     luB.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     llA.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     llB.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     ruA.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     ruB.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     rlA.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     rlB.set_brake_mode(MOTOR_BRAKE_BRAKE);

//     left_rotation_sensor.set_data_rate(5);
//     right_rotation_sensor.set_data_rate(5);

//     while(!left_rotation_sensor.reset());
//     while(!right_rotation_sensor.reset());

//     left_rotation_sensor.set_position(0);
//     right_rotation_sensor.set_position(0);
// }

// void opcontrol()
// {
//     master.clear();
//     while (true){
//         leftX = master.get_analog(ANALOG_LEFT_X);
//         leftY = master.get_analog(ANALOG_LEFT_Y);

//         rightX = master.get_analog(ANALOG_RIGHT_X);
//         rightY = master.get_analog(ANALOG_RIGHT_Y);

//         if(leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0){
//             brake();
//         }
//         else{
//             float magnitude = std::sqrt(leftY * leftY + leftX * leftX);
//             int move_speed = static_cast<int>(magnitude);

//             translationL = move_speed;
//             translationR = move_speed;

//             float directionR = -getAngle(rightY, rightX);
//             directionR = round(directionR * 57.2958);

//             float directionL = -getAngle(leftY, leftX);
//             directionL = round(directionR * 57.2958);

//             setDirection(directionL);

//             luA.move_velocity(-translationL + left_turn_speed);
//             luB.move_velocity(-translationL + left_turn_speed);
//             llA.move_velocity(-translationL - left_turn_speed);
//             llB.move_velocity(-translationL - left_turn_speed);

//             ruA.move_velocity(translationR + right_turn_speed);
//             ruB.move_velocity(translationR + right_turn_speed);
//             rlA.move_velocity(translationR - right_turn_speed);
//             rlB.move_velocity(translationR - right_turn_speed);
//         }
        
//         pros::delay(20);
//     }
// }