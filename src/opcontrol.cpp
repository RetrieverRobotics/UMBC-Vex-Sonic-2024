/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "umbc.h"

#include <cstdint>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true

// ports for left drive
#define LEFT_FRONT_MOTOR_PORT 3
#define LEFT_BACK_MOTOR_PORT  4

// ports for right drive
#define RIGHT_FRONT_MOTOR_PORT 1
#define RIGHT_BACK_MOTOR_PORT  2

// ports for lift
#define LIFT_MOTOR_PORT 19

// ports for wings
#define WING_LEFT_MOTOR_PORT  6
#define WING_RIGHT_MOTOR_PORT 18

#define WING_POSITION_MAX_ERROR 10
#define WING_POSITION_EXPAND    2000


void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize left drive
    pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT);
	pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT, MOTOR_REVERSE);
    pros::MotorGroup drive_left = pros::MotorGroup(vector<pros::Motor>{drive_left_back_motor, drive_left_back_motor});
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_GREEN);
	
    // initialize right drive
    pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);
	pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT, MOTOR_REVERSE);
    pros::MotorGroup drive_right = pros::MotorGroup(vector<pros::Motor>{drive_right_back_motor, drive_right_back_motor});
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_GREEN);

    // initialize lift
    pros::Motor lift_motor = pros::Motor(LIFT_MOTOR_PORT, MOTOR_REVERSE);
    pros::MotorGroup lift = pros::MotorGroup(vector<pros::Motor>{lift_motor});
    lift.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    lift.set_gearing(E_MOTOR_GEAR_RED);

    // initialize wings
    bool is_wing_position_manual = false;
    static double wing_position = 0;
    static double wing_position_target = 0;
	pros::Motor wing_left_motor = pros::Motor(WING_LEFT_MOTOR_PORT);
	pros::Motor wing_right_motor = pros::Motor(WING_RIGHT_MOTOR_PORT);
    pros::MotorGroup wings = pros::MotorGroup(vector<pros::Motor>{wing_right_motor, wing_left_motor});
    wings.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    wings.set_gearing(E_MOTOR_GEAR_BLUE);
    
    // set zero position for wings
    if (0 == wing_position) {
        wings.tare_position();
    } else {
        wings.set_zero_position(-wing_position);
    }

    while(1) {

        // set velocity for drive (arcade controls)
        int32_t arcade_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t arcade_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);

        int32_t drive_left_velocity = (int32_t)(((double)(arcade_x - arcade_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);

        int32_t drive_right_velocity = (int32_t)(((double)(arcade_x + arcade_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);                                

        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        // set lift position
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_L1)) {
            lift.move_velocity(MOTOR_RED_GEAR_MULTIPLIER);
        } else if (controller_master->get_digital(E_CONTROLLER_DIGITAL_L2)) {
            lift.move_velocity(-MOTOR_RED_GEAR_MULTIPLIER);
        } else {
            lift_motor.brake();
        }

        // set position for wings
        wing_position = wings.get_positions().front();
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
            wing_position_target = (wing_position_target == WING_POSITION_EXPAND) ? 0 : WING_POSITION_EXPAND;
            wings.move_absolute(wing_position_target, MOTOR_BLUE_GEAR_MULTIPLIER);
            is_wing_position_manual = false;
        }
        
        // manually control left wing
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_X)) {
            wing_left_motor.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
            is_wing_position_manual = true;
        } else if (controller_master->get_digital(E_CONTROLLER_DIGITAL_Y)) {
            wing_left_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
            is_wing_position_manual = true;
        } else if (is_wing_position_manual) {
            wing_left_motor.brake();
        }
    
        // manually control right wing
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_A)) {
            wing_right_motor.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
            is_wing_position_manual = true;
        } else if (controller_master->get_digital(E_CONTROLLER_DIGITAL_B)) {
            wing_right_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
            is_wing_position_manual = true;
        } else if (is_wing_position_manual) {
            wing_right_motor.brake();
        }

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}