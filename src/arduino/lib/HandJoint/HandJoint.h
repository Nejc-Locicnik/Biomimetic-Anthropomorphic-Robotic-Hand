#ifndef HandJoint_h
#define HandJoint_h
class HandJoint {
    public:
        HandJoint(int min_pos, int max_pos, int joint_type);
        
        int     joint_position;
        bool    joint_movement_available;

        void    initFiltering(int joint_pos);
        void    invertServoRotation();
        void    updateJointPosition(int joint_pos);
        bool    movementAvailable(bool joint_1, bool joint_2);
        void    updateSetpoint(int position);
        int     regulateJoint(bool joint_1, bool joint_2);

    private:
        int     _min_pos;
        int     _max_pos;
        int     _joint_type;
        int     _pid_accuracy;
        int     _min_servo_speed;
        int     _max_servo_speed;
        int     _invert_servo_rotation;
        float   _kp;
        float   _ki;
        float   _kd;

        double  _filtering_array[10];
        int     _joint_setpoint;
        int     _error;
        int     _direction;
        int     _last_error;
        double  _cum_error;
        double  _rate_error;
        double  _pid_value;

        unsigned long   _last_correction_time;

        void    shiftFiltering(int joint_pos);
        void    calcFiltering();

        void    calcError();
        void    updateDirection();
        void    updateJointMovementAvailability();
        int     calcPID();
};
#endif


