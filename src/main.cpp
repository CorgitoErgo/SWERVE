#include "main.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);

void brake(){
    luA.brake();
    luB.brake();
    llA.brake();
    llB.brake();
    ruA.brake();
    ruB.brake();
    rlA.brake();
    rlB.brake();
    pros::delay(1);
}

int getNormalizedSensorAngle(pros::Rotation &sensor)
{
    float angle = sensor.get_angle() / 100.0; // Convert from centidegrees to degrees

    if (angle < -186)
        angle += 360;
    else if (angle > 186)
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
    if (angle > 180.0) {
        return angle -= 360;
    } else if (angle < -180.0) {
        return angle += 360;
    } else {
        return angle;
    }
}

bool reverseDriveL = false;
bool reverseDriveR = false;
int absoluteAngleL = 0;
int absoluteAngleR = 0;

void set_wheel_angle(){
    while(1){
        // float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
        // float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

        // // Calculate error between current and target angle
        // float left_error = target_angle - left_current_angle;
        // float right_error = target_angle - right_current_angle;

        // // Normalize error to be within -180 to 180 degrees
        // while (left_error > 180) left_error -= 360;
        // while (left_error < -180) left_error += 360;
        // while (right_error > 180) right_error -= 360;
        // while (right_error < -180) right_error += 360;

        // // Decide whether to rotate the wheel or reverse the drive
        // if (fabs(left_error) > 90) {
        //     // Reverse the drive direction
        //     left_error -= 180;
        //     if (left_error < -180) left_error += 360;
        // }

        // if (fabs(right_error) > 90) {
        //     // Reverse the drive direction
        //     right_error -= 180;
        //     if (right_error < -180) right_error += 360;
        // }

        float left_previous_error = 0;
        float right_previous_error = 0;

        float left_integral = 0;
        float right_integral = 0;

        while(setAngle){
            float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
            float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

            float left_error = wrapAngle(target_angle - left_current_angle);
            float right_error = wrapAngle(target_angle - right_current_angle);

            // Normalize error again after updating current angles
            while (left_error > 180) left_error -= 360;
            while (left_error < -180) left_error += 360;
            while (right_error > 180) right_error -= 360;
            while (right_error < -180) right_error += 360;

            // Recheck if we need to reverse the drive
            if (fabs(left_error) > 90) {
                left_error -= 180;
                if (left_error < -180){ 
                    left_error += 360;
                }
            }

            if (fabs(right_error) > 90) {
                right_error -= 180;
                if (right_error < -180) {
                    right_error += 360;
                }
            }

            left_integral += left_error;
            right_integral += right_error;

            float left_derivative = left_error - left_previous_error;
            float right_derivative = right_error - right_previous_error;

            left_previous_error = left_error;
            right_previous_error = right_error;

            float left_motor_speed = lkP * left_error + lkI * left_integral + lkD * left_derivative;
            float right_motor_speed = rkP * right_error + rkI * right_integral + rkD * right_derivative;

            left_turn_speed = left_motor_speed*0.7;
            right_turn_speed = right_motor_speed*0.7;

            if(fabs(left_error) <= 5){
                left_turn_speed = 0;
            }
            if(fabs(right_error) <= 5){
                right_turn_speed = 0;
            }
            if(fabs(left_error) <= 5 && fabs(right_error) <= 5){
                left_turn_speed = 0;
                right_turn_speed = 0;
                setAngle = false;
                break;
            }

            pros::delay(2);
        }
        pros::delay(5);
    }
}

int translation = 0;
int translationL = 0;
int translationR = 0;

void SwerveTranslation(){
    while(1){
        float magnitude = std::sqrt(leftY * leftY + leftX * leftX);
        float direction = -getAngle(leftY, leftX);
        int move_speed = static_cast<int>(magnitude);
        direction = round(direction * 57.2958); // Convert to degrees

        if(direction < 10000 && magnitude >= 5){
            float left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
            float right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);

            std::cout << "directionL: " << left_sensor_angle << std::endl;
            std::cout << "directionR: " << right_sensor_angle << std::endl;

            std::cout << "RevL: " << reverseDriveL << std::endl;
            std::cout << "RevR: " << reverseDriveR << std::endl;

            if (direction > 0)
                other_angle = direction - 180;
            else
                other_angle = direction + 180;

            // float left_error = fabs(direction - left_sensor_angle);
            // float right_error = fabs(direction - right_sensor_angle);
            // float left_error_other = fabs(other_angle - left_sensor_angle);
            // float right_error_other = fabs(other_angle - right_sensor_angle);

            if(other_angle == 180){
                other_angle -= 10;
                left_sensor_angle -= 10;
                right_sensor_angle -= 10;
            }
            else if(other_angle == 0){
                other_angle += 10;
                left_sensor_angle += 10;
                right_sensor_angle += 10;
            }

            target_angle = other_angle;
            setAngle = true;
            std::cout << "direction: " << other_angle << std::endl;


            // if(int(other_angle) != 90 || int(other_angle) != -90){
            //     if(other_angle <= 95 && other_angle >= -85){
            //         if(left_sensor_angle <= 105 && left_sensor_angle >= -105)
            //             translationL = move_speed;
            //         else
            //             translationL = -move_speed;

            //         if(right_sensor_angle <= 105 && right_sensor_angle >= -105)
            //             translationR = move_speed;
            //         else
            //             translationR = -move_speed;
            //     }
            //     else{
            //         if(left_sensor_angle <= 105 && left_sensor_angle >= -105)
            //             translationL = -move_speed;
            //         else
            //             translationL = move_speed;

            //         if(right_sensor_angle <= 105 && right_sensor_angle >= -105)
            //             translationR = -move_speed;
            //         else
            //             translationR = move_speed;
            //     }
            // }

            // if((int(other_angle) < 80 && int(other_angle) >= 0) || (int(other_angle) < 0 && int(other_angle) > -80)){
            //     if(left_sensor_angle <= 85 && left_sensor_angle >= -95)
            //         translationL = move_speed;
            //     else
            //         translationL = -move_speed;

            //     if(right_sensor_angle <= 85 && right_sensor_angle >= -95)
            //         translationR = move_speed;
            //     else
            //         translationR = -move_speed;
            //     pros::delay(100);
            // }
            // else if((int(other_angle) > 115 && int(other_angle) <= 180) || (int(other_angle) < -90 && int(other_angle) > -180)){
            //     if(left_sensor_angle <= 85 && left_sensor_angle >= -95)
            //         translationL = -move_speed;
            //     else
            //         translationL = move_speed;

            //     if(right_sensor_angle <= 85 && right_sensor_angle >= -95)
            //         translationR = -move_speed;
            //     else
            //         translationR = move_speed;
            //     pros::delay(100);
            // }
            // else{
            //     translationL = -move_speed*(direction*0.01)*(left_sensor_angle*0.01);
            //     translationR = -move_speed*(direction*0.01)*(right_sensor_angle*0.01);
            // }

            translationL = -move_speed;
            translationR = -move_speed;

            if(left_sensor_angle < 0)
                translationL *= -1;
            if(right_sensor_angle < 0)
                translationR *= -1;
            if(other_angle > 0){
                translationL *= -1;
                translationR *= -1;
            }

            // if(direction <= 100 && direction >= -85){
            //     if(left_sensor_angle <= 100 && left_sensor_angle >= -85)
            //         translationL = -move_speed;
            //     else
            //         translationL = move_speed;

            //     if(right_sensor_angle <= 100 && right_sensor_angle >= -85)
            //         translationR = -move_speed;
            //     else
            //         translationR = move_speed;
            // }
            // else{
            //     if(left_sensor_angle <= 100 && left_sensor_angle >= -85)
            //         translationL = move_speed;
            //     else
            //         translationL = -move_speed;

            //     if(right_sensor_angle <= 100 && right_sensor_angle >= -85)
            //         translationR = move_speed;
            //     else
            //         translationR = -move_speed;
            // }

            // if(left_sensor_angle > 0)
            //     translationL = -move_speed;
            // else
            //     translationL = move_speed;

            // if(right_sensor_angle > 0)
            //     translationR = -move_speed;
            // else
            //     translationR = move_speed;
            

            // luA.move_velocity(move_speed + left_turn_speed - rightX);
            // luB.move_velocity(move_speed + left_turn_speed - rightX);
            // llA.move_velocity(move_speed - left_turn_speed - rightX);
            // llB.move_velocity(move_speed - left_turn_speed - rightX);

            // ruA.move_velocity(-move_speed + right_turn_speed - rightX);
            // ruB.move_velocity(-move_speed + right_turn_speed - rightX);
            // rlA.move_velocity(-move_speed - right_turn_speed - rightX);
            // rlB.move_velocity(-move_speed - right_turn_speed - rightX);
            pros::delay(2);
        }
        else{
            move_speed = 0;
            translation = 0;
            translationL = 0;
            translationR = 0;
        }

        pros::delay(2);
    }
}

void SwerveRotation(){
    while(1){
        if(abs(rightX) > 0){
                left_turn_speed *= 0.8;
                right_turn_speed *= 0.8;
                target_angle = 45;
                setAngle = true;

                // luA.move(left_turn_speed - rightX);
                // luB.move(left_turn_speed - rightX);
                // llA.move(-left_turn_speed - rightX);
                // llB.move(-left_turn_speed - rightX);

                // ruA.move(right_turn_speed - rightX);
                // ruB.move(right_turn_speed - rightX);
                // rlA.move(- right_turn_speed - rightX);
                // rlB.move(- right_turn_speed - rightX);
        }
        pros::delay(10);
    }
}

void initialize(){
    pros::lcd::initialize();

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

    std::cout << left_rotation_sensor.get_angle() << std::endl;
    std::cout << right_rotation_sensor.get_angle() << std::endl;

    std::cout << getNormalizedSensorAngle(left_rotation_sensor) << std::endl;
    std::cout << getNormalizedSensorAngle(right_rotation_sensor) << std::endl;

    pros::Task translate(SwerveTranslation);
    pros::Task wheel_angle(set_wheel_angle);
    pros::Task rotation(SwerveRotation);
    
    wheel_angle.set_priority(1);
    rotation.set_priority(2);
    translate.set_priority(3);
}

void opcontrol()
{
    while (true){
        leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);
        rightY = master.get_analog(ANALOG_RIGHT_Y);

        if(leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0){
            brake();
        }

        luA.move_velocity(-translationL + left_turn_speed - rightX);
        luB.move_velocity(-translationL + left_turn_speed - rightX);
        llA.move_velocity(-translationL - left_turn_speed - rightX);
        llB.move_velocity(-translationL - left_turn_speed - rightX);

        ruA.move_velocity(translationR + right_turn_speed - rightX);
        ruB.move_velocity(translationR + right_turn_speed - rightX);
        rlA.move_velocity(translationR - right_turn_speed - rightX);
        rlB.move_velocity(translationR - right_turn_speed - rightX);
        
        pros::delay(2);
    }
}   