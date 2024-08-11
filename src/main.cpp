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

        // Calculate error between current and target angle
        float left_error = target_angle - left_current_angle;
        float right_error = target_angle - right_current_angle;

        // Normalize error to be within -180 to 180 degrees
        while (left_error > 180) left_error -= 360;
        while (left_error < -180) left_error += 360;
        while (right_error > 180) right_error -= 360;
        while (right_error < -180) right_error += 360;

        // Decide whether to rotate the wheel or reverse the drive
        if (fabs(left_error) > 90) {
            // Reverse the drive direction
            left_error -= 180;
            if (left_error < -180) left_error += 360;
        }

        if (fabs(right_error) > 90) {
            // Reverse the drive direction
            right_error -= 180;
            if (right_error < -180) right_error += 360;
        }

        float left_previous_error = 0;
        float right_previous_error = 0;

        float left_integral = 0;
        float right_integral = 0;

        while(setAngle){
            left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
            right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

            left_error = target_angle - left_current_angle;
            right_error = target_angle - right_current_angle;

            // Normalize error again after updating current angles
            while (left_error > 180) left_error -= 360;
            while (left_error < -180) left_error += 360;
            while (right_error > 180) right_error -= 360;
            while (right_error < -180) right_error += 360;

            // Recheck if we need to reverse the drive
            if (fabs(left_error) > 90) {
                left_error -= 180;
                //if (left_error < -180) left_error += 360;
            }

            if (fabs(right_error) > 90) {
                right_error -= 180;
                //if (right_error < -180) right_error += 360;
            }

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
                left_turn_speed = 0;
                right_turn_speed = 0;
                setAngle = false;
                break;
            }

            pros::delay(5);
        }
        pros::delay(10);
    }
}

int translation = 0;

void SwerveTranslation(){
    while(1){
        float magnitude = std::sqrt(leftY * leftY + leftX * leftX);
        float direction = -getAngle(leftY, leftX);
        int move_speed = static_cast<int>(magnitude);
        direction = round(direction * 57.2958); // Convert to degrees

        if(direction < 10000 && magnitude >= 5){
            std::cout << "direction: " << direction << std::endl;

            // float left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
            // float right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);
            if (direction > 0)
                other_angle = direction - 180;
            else
                other_angle = direction + 180;

            // float left_error = fabs(direction - left_sensor_angle);
            // float right_error = fabs(direction - right_sensor_angle);
            // float left_error_other = fabs(other_angle - left_sensor_angle);
            // float right_error_other = fabs(other_angle - right_sensor_angle);

            target_angle = direction;
            setAngle = true;

            translation = move_speed;

            // luA.move_velocity(move_speed + left_turn_speed - rightX);
            // luB.move_velocity(move_speed + left_turn_speed - rightX);
            // llA.move_velocity(move_speed - left_turn_speed - rightX);
            // llB.move_velocity(move_speed - left_turn_speed - rightX);

            // ruA.move_velocity(-move_speed + right_turn_speed - rightX);
            // ruB.move_velocity(-move_speed + right_turn_speed - rightX);
            // rlA.move_velocity(-move_speed - right_turn_speed - rightX);
            // rlB.move_velocity(-move_speed - right_turn_speed - rightX);
        }
        else{
            move_speed = 0;
            translation = 0;
        }

        pros::delay(5);
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
        pros::delay(5);
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
    left_rotation_sensor.reset();
    right_rotation_sensor.reset();

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

        luA.move_velocity(translation + left_turn_speed - rightX);
        luB.move_velocity(translation + left_turn_speed - rightX);
        llA.move_velocity(translation - left_turn_speed - rightX);
        llB.move_velocity(translation - left_turn_speed - rightX);

        ruA.move_velocity(-translation + right_turn_speed - rightX);
        ruB.move_velocity(-translation + right_turn_speed - rightX);
        rlA.move_velocity(-translation - right_turn_speed - rightX);
        rlB.move_velocity(-translation - right_turn_speed - rightX);
        
        pros::delay(20);
    }
}