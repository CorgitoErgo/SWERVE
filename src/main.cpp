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
    pros::delay(2);
}

float getNormalizedSensorAngle(pros::Rotation &sensor)
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

void set_wheel_angle(){
    while(1){
        float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
        float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

        if (target_angle > 0)
        {
            other_angle = target_angle - 180;
        }
        else
        {
            other_angle = target_angle + 180;
        }

        if (fabs(target_angle - left_current_angle) < fabs(other_angle - left_current_angle))
            target_angle = target_angle;
        else
            target_angle = other_angle;

        float left_error = target_angle - left_current_angle;
        float right_error = target_angle - right_current_angle;

        float left_previous_error = 0;
        float right_previous_error = 0;

        float left_integral = 0;
        float right_integral = 0;

        // Normalize the errors to be within [-180, 180] degrees
        while (left_error > 180)
            left_error -= 360;
        while (left_error < -180)
            left_error += 360;
        while (right_error > 180)
            right_error -= 360;
        while (right_error < -180)
            right_error += 360;

        while(setAngle){
            left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
            right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

            left_error = target_angle - left_current_angle;
            right_error = target_angle - right_current_angle;

            while (left_error > 180)
                left_error -= 360;
            while (left_error < -180)
                left_error += 360;
            while (right_error > 180)
                right_error -= 360;
            while (right_error < -180)
                right_error += 360;

            left_integral += left_error;
            right_integral += right_error;

            float left_derivative = left_error - left_previous_error;
            float right_derivative = right_error - right_previous_error;

            left_previous_error = left_error;
            right_previous_error = right_error;

            float left_motor_speed = kP * left_error + kI * left_integral + kD * left_derivative;
            float right_motor_speed = kP * right_error + kI * right_integral + kD * right_derivative;

            left_turn_speed = left_motor_speed;
            right_turn_speed = right_motor_speed;

            if(fabs(left_error) <= 5){
                left_turn_speed = 0;
            }
            if(fabs(right_error) <= 5){
                right_turn_speed = 0;
            }
            if(fabs(left_error) <= 5 && fabs(right_error) <= 5){
                setAngle = false;
                break;
            }

            pros::delay(5);
        }
        pros::delay(20);
    }
}

void SwerveTranslation(){
    while(1){
        float magnitude = std::sqrt(leftY * leftY + leftX * leftX);
        float direction = -getAngle(leftY, leftX);
        float rotate = getAngle(rightY, rightX);

        if(direction < 10000 && magnitude >= 5){
            direction = round(direction * 57.2958); // Convert to degrees
            int move_speed = static_cast<int>(magnitude);
            float left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
            float right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);
            if (direction > 0)
                other_angle = direction - 180;
            else
                other_angle = direction + 180;

            float left_error = fabs(direction - left_sensor_angle);
            float right_error = fabs(direction - right_sensor_angle);
            float left_error_other = fabs(other_angle - left_sensor_angle);
            float right_error_other = fabs(other_angle - right_sensor_angle);

            target_angle = direction;
            setAngle = true;

            luA.move_velocity(move_speed + left_turn_speed - rightX);
            luB.move_velocity(move_speed + left_turn_speed - rightX);
            llA.move_velocity(move_speed - left_turn_speed - rightX);
            llB.move_velocity(move_speed - left_turn_speed - rightX);

            ruA.move_velocity(-move_speed + right_turn_speed - rightX);
            ruB.move_velocity(-move_speed + right_turn_speed - rightX);
            rlA.move_velocity(-move_speed - right_turn_speed - rightX);
            rlB.move_velocity(-move_speed - right_turn_speed - rightX);
        }
        else{
            brake();
        }
        if(abs(rightX) > 0){
            luA.move_velocity(-rightX);
            luB.move_velocity(-rightX);
            llA.move_velocity(-rightX);
            llB.move_velocity(-rightX);

            ruA.move_velocity(-rightX);
            ruB.move_velocity(-rightX);
            rlA.move_velocity(-rightX);
            rlB.move_velocity(-rightX);
        }

        pros::delay(10);
    }
}

void initialize(){
    pros::lcd::initialize();

    luA.set_brake_mode(MOTOR_BRAKE_COAST);
    luB.set_brake_mode(MOTOR_BRAKE_COAST);
    llA.set_brake_mode(MOTOR_BRAKE_COAST);
    llB.set_brake_mode(MOTOR_BRAKE_COAST);
    ruA.set_brake_mode(MOTOR_BRAKE_COAST);
    ruB.set_brake_mode(MOTOR_BRAKE_COAST);
    rlA.set_brake_mode(MOTOR_BRAKE_COAST);
    rlB.set_brake_mode(MOTOR_BRAKE_COAST);

    pros::Task translate(SwerveTranslation);
    pros::Task wheel_angle(set_wheel_angle);
}

void opcontrol()
{
    while (true){
        leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);
        rightY = master.get_analog(ANALOG_RIGHT_Y);
        
        pros::delay(20);
    }
}