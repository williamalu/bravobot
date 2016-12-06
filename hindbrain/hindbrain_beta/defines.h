//Since the cmd_vel speeds range between -1 and 1, the following variables simplify
//the math needed to convert these speeds to the 24-90-156 speeds. Feel free to hard-code these in. 
#define MOTOR_OFFSET 90.0
#define MOTOR_MULTIPLIER -66.0
#define MOTOR_STOP 90

#define ROS_PUBLISH_INTERVAL 500

#define OK_MODE 0
#define PHYS_ESTOP_MODE 1
#define IR_DETECTED_MODE 2
