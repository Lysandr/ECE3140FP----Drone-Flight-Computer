/****************************
*
* Padraig Lysandrou
* Eric Berg
* Drone Flight Controller
* ECE3140 Final Project
*
*
*
*
*
******************************/




float p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float p_gain_pitch = p_gain_roll;  //Gain setting for the pitch P-controller.
float i_gain_pitch = i_gain_roll;  //Gain setting for the pitch I-controller.
float d_gain_pitch = d_gain_roll;  //Gain setting for the pitch D-controller.
int max_pitch = max_roll;          //Maximum output of the PID-controller (+/-)

float p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)











