//ROS
ros::NodeHandle  nh;

void cmdvel_cb( const geometry_msgs::Twist& msg); //callback prototype, find definition in ROS.ino
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmdvel_cb);

std_msgs::Int16 portspeed_msg;
std_msgs::Int16 starboardspeed_msg;
ros::Publisher pub_portspeed("motor/port", &portspeed_msg);
ros::Publisher pub_starboardspeed("motor/starboard", &starboardspeed_msg);

//Motors
Servo port_motors;
Servo starboard_motors;
int port_speed = MOTOR_STOP; // range from 24 to 156
int starboard_speed = MOTOR_STOP; // range from 24 to 156

//Time
long lastPublishTime = 0;

uint8_t mode = 0;




