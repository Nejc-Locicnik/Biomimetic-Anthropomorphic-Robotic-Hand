#include "Arduino.h"
#include "HandConstants.h"
#include "HandJoint.h"

HandJoint::HandJoint(int min_pos, int max_pos, int joint_type) {
    _joint_type             = joint_type;
    _min_pos                = min_pos;
    _max_pos                = max_pos;
    _pid_accuracy           = PID_ACCURACY;
    _min_servo_speed        = 1500 - (int)(1000 * SERVO_SPEED_CAP);                
    _max_servo_speed        = 1500 + (int)(1000 * SERVO_SPEED_CAP);
    _kp                     = KP;
    _ki                     = KI;
    _kd                     = KD;
    _invert_servo_rotation  = 1;
}

void HandJoint::invertServoRotation() {
    _invert_servo_rotation = -1;
}

void HandJoint::initFiltering(int joint_pos) {
    _filtering_array[0] = joint_pos;
    _filtering_array[1] = joint_pos;
    _filtering_array[2] = joint_pos;
    _filtering_array[3] = joint_pos;
    _filtering_array[4] = joint_pos;
    _filtering_array[5] = joint_pos;
    _filtering_array[6] = joint_pos;
    _filtering_array[7] = joint_pos;
    _filtering_array[8] = joint_pos;
    _filtering_array[9] = joint_pos;

    joint_position = joint_pos;
}

void HandJoint::shiftFiltering(int joint_pos) {
    _filtering_array[9] = _filtering_array[8];
    _filtering_array[8] = _filtering_array[7];
    _filtering_array[7] = _filtering_array[6];
    _filtering_array[6] = _filtering_array[5];
    _filtering_array[5] = _filtering_array[4];
    _filtering_array[4] = _filtering_array[3];
    _filtering_array[3] = _filtering_array[2];
    _filtering_array[2] = _filtering_array[1];
    _filtering_array[1] = _filtering_array[0];
    _filtering_array[0] = joint_pos;
}

void HandJoint::calcFiltering() {
    joint_position = (int)((_filtering_array[0]*10 + _filtering_array[1]* 9 + _filtering_array[2]* 8 + _filtering_array[3]* 7 + 
                           _filtering_array[4]* 6 + _filtering_array[5]* 5 + _filtering_array[6]* 4 + _filtering_array[7]* 3 + 
                           _filtering_array[8]* 2 + _filtering_array[9]* 1)/55);
}

void HandJoint::updateJointPosition(int joint_pos) {
    shiftFiltering(joint_pos);
    calcFiltering();
}

bool HandJoint::movementAvailable(bool joint_1, bool joint_2) {
    if (_joint_type == 1) {
        return joint_movement_available && joint_1 && joint_2;     // MCP joint needs to also check PIP and DIP joints
    } else if (_joint_type == 2) {
        return joint_movement_available && joint_2;                // PIP joint needs to also check DIP joint
    } else {
        return joint_movement_available;                           // MCPR and DIP joints only need to check themselves
    }
}

void HandJoint::updateSetpoint(int position) {
    _joint_setpoint = position;
    _cum_error = 0;
}

// check if the joint is near its min/max boundaries, returns false if movement can go over boundaries, otherwise returns true
// to prevent joints from getting stuck we check direction of movement and only block movement towards the boundary
void HandJoint::updateJointMovementAvailability() {
    if (_direction == -1) {
        joint_movement_available = !(joint_position + 15 > _max_pos);
    } else if (_direction == 1) {
        joint_movement_available = !(joint_position - 15 < _min_pos);
    } else {
        joint_movement_available = 1;
    }
}

void HandJoint::calcError() {
    _error = joint_position - _joint_setpoint;
}

void HandJoint::updateDirection() {
    if (_error > 0) {
        _direction = 1 * _invert_servo_rotation;
    } else {
        _direction = -1 * _invert_servo_rotation;
    }
}

int HandJoint::calcPID() {
    unsigned long   current_time;
    double          delta;

    current_time = millis();
    delta = current_time - _last_correction_time;

    if ((abs(_error) - abs(_last_error)) > (_error - _last_error)) {
        _cum_error = 0;
    }

    _cum_error  += _error * delta;
    _rate_error  = (_error - _last_error) / delta;
    _pid_value   = 1500 + (_direction * abs((_kp * _error) + (_ki * _cum_error) + (_kd * _rate_error)));

    // Cap servo speed value in the available range
    if (_pid_value > _max_servo_speed) {
        _pid_value = _max_servo_speed;
    } else if (_pid_value < _min_servo_speed) {
        _pid_value = _min_servo_speed;
    }

    _last_correction_time = current_time;
    _last_error = _error;
    return (int)_pid_value;
}

int HandJoint::regulateJoint(bool joint_1, bool joint_2) {
    calcError();
    updateDirection();
    updateJointMovementAvailability();
    if ((abs(_error) > _pid_accuracy) && movementAvailable(joint_1, joint_2)) {
        return calcPID();
    } else {
        return (int)1500;
    }
}