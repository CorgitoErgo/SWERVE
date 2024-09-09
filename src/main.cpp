#include "main.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);

LV_IMG_DECLARE(sus);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void brake(){
    luA.brake();
    luB.brake();
    llA.brake();
    llB.brake();
    ruA.brake();
    ruB.brake();
    rlA.brake();
    rlB.brake();
    pros::delay(2);
}

double apply_deadband(double value){
    return (fabs(value) > DEADBAND) ? value : 0.0;
}

//Limit values within the max RPM
double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

double closestAngle(double a, double b)
{
        // get direction
        double dir = std::fmod(b, 360.0) - std::fmod(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (abs(dir) > 180.0)
            dir = -(sgn(dir) * 360.0) + dir;

        return dir;
}

int getNormalizedSensorAngle(pros::Rotation &sensor)
{
    float angle = sensor.get_angle() / 100.0; // Convert from centidegrees to degrees

    if (angle < -180)
        angle += 360;
    else if (angle > 180)
        angle -= 360;

    return angle;
}

double getAngle(int x, int y)
{
    double a = std::atan2(std::abs(y), std::abs(x));

    if (x < 0){
        if (y < 0)
            return (M_PI * -1) + a;
        else if (y > 0)
            return M_PI - a;
        else
            return M_PI;
    }
    else if (x > 0){
        if (y < 0)
            return a * -1;
        else if (y > 0)
            return a;
        else
            return 0;
    }
    else{
        if (y < 0)
            return -M_PI / 2;
        else if (y > 0)
            return M_PI / 2;
        else
            return 10000; // return illegal value because a zero vector has no direction
    }
}

double wrapAngle(double angle) {
    if (angle > 180.0)
        return angle -= 360;
    else if (angle < -180.0)
        return angle += 360;
    else
        return angle;
}

bool reverseDriveL = false;
bool reverseDriveR = false;
int absoluteAngleL = 0;
int absoluteAngleR = 0;
float target_angleL = 0;
float target_angleR = 0;

void set_wheel_angle(){
    while(1){
        float left_previous_error = 0;
        float right_previous_error = 0;
        float left_integral = 0;
        float right_integral = 0;

        while(setAngle){
            float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
            float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

            float left_error = wrapAngle((target_angleL + (rightX != 0 ? (rightX > 0 ? 10 : -10) : 0)) - left_current_angle);
            float right_error = wrapAngle((target_angleR + (rightX != 0 ? (rightX > 0 ? 10 : -10) : 0)) - right_current_angle);

            left_integral += left_error;
            right_integral += right_error;

            float left_derivative = left_error - left_previous_error;
            float right_derivative = right_error - right_previous_error;

            left_previous_error = left_error;
            right_previous_error = right_error;

            float left_motor_speed = lkP * left_error + lkI * left_integral + lkD * left_derivative;
            float right_motor_speed = rkP * right_error + rkI * right_integral + rkD * right_derivative;

            left_turn_speed = left_motor_speed;
            right_turn_speed = right_motor_speed;

            if(fabs(left_error) <= 1){
                left_turn_speed = 0;
            }
            if(fabs(right_error) <= 1){
                right_turn_speed = 0;
            }
            if(fabs(left_error) <= 1 && fabs(right_error) <= 1){
                left_turn_speed = 0;
                right_turn_speed = 0;
                setAngle = false;
                break;
            }
            pros::delay(5);
        }
        pros::delay(5);
    }
}

int translation = 0;
int translationL = 0;
int translationR = 0;
float prevDirection = 0;
double shortestAngle = 0;
float leftDirection = 0;
bool leftflip = false;
bool rightflip = false;
int currDirection = 0;

void SwerveTranslation(){
    while(1){
        float magnitude = std::sqrt(leftY * leftY + leftX * leftX) * SCALING_FACTOR;
        int move_speed = static_cast<int>(magnitude);

        float direction = -getAngle(leftY, leftX);
        direction = -round(direction * 57.2958);
        
        leftDirection = direction;
        translationL = bound_value(move_speed);
        translationR = bound_value(move_speed);
        currDirection = direction;

        if(direction < 10000 && magnitude > 5){
            float left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
            float right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);

            double setpointAngleL = closestAngle(left_sensor_angle, direction);
            double setpointAngleFlippedL = closestAngle(left_sensor_angle, direction + 180.0);

            double setpointAngleR = closestAngle(right_sensor_angle, direction);
            double setpointAngleFlippedR = closestAngle(right_sensor_angle, direction + 180.0);

            if (abs(setpointAngleL) <= abs(setpointAngleFlippedL)){
            // unflip the motor direction use the setpoint
                if(translationL < 0){
                    leftflip = false;
                    translationL = -translationL;
                }
                //target_angle = (left_sensor_angle + setpointAngle);
                if(rightX == 0 && rightY == 0)
                    target_angleL = (left_sensor_angle + setpointAngleL);
            }
            // if the closest angle to setpoint + 180 is shorter
            else{
                // flip the motor direction and use the setpoint + 180
                if(translationL > 0){
                    leftflip = true;    
                    translationL = -translationL;
                }
                //target_angle = (left_sensor_angle + setpointAngleFlipped);
                if(rightX == 0 && rightY == 0)
                    target_angleL = (left_sensor_angle + setpointAngleFlippedL);
            }

            if (abs(setpointAngleR) <= abs(setpointAngleFlippedR)){
                // unflip the motor direction use the setpoint
                if(translationR < 0){
                    rightflip = false;
                    translationR = -translationR;
                }
                //target_angle = (left_sensor_angle + setpointAngle);
                if(rightX == 0 && rightY == 0)
                    target_angleR = (right_sensor_angle + setpointAngleR);
            }
            // if the closest angle to setpoint + 180 is shorter
            else{
                // flip the motor direction and use the setpoint + 180
                if(translationR > 0){
                    rightflip = true;
                    translationR = -translationR;
                }
                //target_angle = (left_sensor_angle + setpointAngleFlipped);
                if(rightX == 0 && rightY == 0)
                    target_angleR = (right_sensor_angle + setpointAngleFlippedR);
            }

            setAngle = true;
        }

        pros::delay(5);
    }
}

void initialize(){
    pros::lcd::clear();
    //lv_obj_t* img = lv_img_create(lv_scr_act(), NULL);
    //lv_img_set_src(img, &sus);
    //lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);

    luA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    luB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    llA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    llB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    ruA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    ruB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rlA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rlB.set_brake_mode(MOTOR_BRAKE_BRAKE);

    left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

    while(!left_rotation_sensor.reset());
    while(!right_rotation_sensor.reset());

    left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

    pros::Task translate(SwerveTranslation);
    pros::Task wheel_angle(set_wheel_angle);
    
    wheel_angle.set_priority(1);
    translate.set_priority(2);
}

int rot_speed = 0;

void opcontrol()
{
    master.clear();
    while (true){
        leftX = apply_deadband(master.get_analog(ANALOG_LEFT_X));
        leftY = apply_deadband(master.get_analog(ANALOG_LEFT_Y));

        rightX = apply_deadband(master.get_analog(ANALOG_RIGHT_X));
        rightY = master.get_analog(ANALOG_RIGHT_Y);

        if(leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0){
            brake();
        }

        std::cout << "leftflip " << translationL << std::endl;
        std::cout << "rightflip " << translationR << std::endl;

        // luA.move_velocity(-translationL + rightX + left_turn_speed);
        // luB.move_velocity(-translationL + rightX + left_turn_speed);
        // llA.move_velocity(-translationL + rightX - left_turn_speed);
        // llB.move_velocity(-translationL + rightX - left_turn_speed);

        // ruA.move_velocity(translationR + rightX + right_turn_speed);
        // ruB.move_velocity(translationR + rightX + right_turn_speed);
        // rlA.move_velocity(translationR + rightX - right_turn_speed);
        // rlB.move_velocity(translationR + rightX - right_turn_speed);

        if(leftX !=0 || leftY != 0){
            int powerL = bound_value(translationL + (rightX * SCALING_FACTOR));
            int turnL = bound_value(left_turn_speed * SCALING_FACTOR);
            int powerR = bound_value(translationR - (rightX * SCALING_FACTOR));
            int turnR = bound_value(right_turn_speed * SCALING_FACTOR);

            luA.move_velocity(-powerL - turnL);
            luB.move_velocity(-powerL - turnL);
            llA.move_velocity(-powerL + turnL);
            llB.move_velocity(-powerL + turnL);

            ruA.move_velocity(-powerR - turnR);
            ruB.move_velocity(-powerR - turnR);
            rlA.move_velocity(-powerR + turnR);
            rlB.move_velocity(-powerR + turnR);
        }
        else if(rightX != 0 && leftX == 0 && leftY == 0){
            rightX = bound_value(rightX * SCALING_FACTOR);
            target_angleL = (rightX > 0 ? -10 : 10);
            target_angleR = (rightX > 0 ? -10 : 10);
            setAngle = true;
            luA.move_velocity(-rightX - bound_value(left_turn_speed * SCALING_FACTOR));
            luB.move_velocity(-rightX - bound_value(left_turn_speed * SCALING_FACTOR));
            llA.move_velocity(-rightX + bound_value(left_turn_speed * SCALING_FACTOR));
            llB.move_velocity(-rightX + bound_value(left_turn_speed * SCALING_FACTOR));

            ruA.move_velocity(rightX - bound_value(right_turn_speed * SCALING_FACTOR));
            ruB.move_velocity(rightX - bound_value(right_turn_speed * SCALING_FACTOR));
            rlA.move_velocity(rightX + bound_value(right_turn_speed * SCALING_FACTOR));
            rlB.move_velocity(rightX + bound_value(right_turn_speed * SCALING_FACTOR));
        }

        pros::delay(10);
    }
}