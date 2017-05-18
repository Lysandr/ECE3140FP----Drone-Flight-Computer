/****************************
*
* Padraig Lysandrou
* Eric Berg
* Drone Flight Controller
* ECE3140 Final Project
*
******************************/

#include "mbed.h"
#include <L3G4200D.h>
#include <PwmIn.h>

/* INCLUDE I2C LIBRARY --> https://community.nxp.com/docs/DOC-101385
  INCLUDE PWM LIBRARY --> https://community.nxp.com/docs/DOC-101383
  other resources
  MDK stuff --> http://www.keil.com/pack/doc/mw/General/html/index.html
  SDK doc --> http://www.nxp.com/assets/documents/data/en/reference-manuals/KSDK20APIRM.pdf?&fpsp=1&WT_TYPE=Reference%20Manuals&WT_VENDOR=FREESCALE&WT_FILE_FORMAT=pdf&WT_ASSET=Documentation&fileExt=.pdf
  CMSIS shit --> http://www.keil.com/pack/doc/CMSIS/Driver/html/group__i2c__interface__gr.html
  MBED libararies --> https://developer.mbed.org/users/mbed_official/code/mbed/
          --> https://docs.mbed.com/docs/mbed-os-handbook/en/latest/getting_started/blinky_compiler/
          --> http://www2.keil.com/mbed
          --> https://developer.mbed.org/handbook/CMSIS-DAP-MDK

  interrupts for the receiver --> interrupting
*/

//PID gains for each of the rate controllers. notice the integral controller is very small
float p_gain_r = 1;
float i_gain_r = 0.04;
float d_gain_r = 13;
float p_gain_p = 1.5;
float i_gain_p = 0;
float d_gain_p = 15;
float p_gain_y = 2;
float i_gain_y = 0.0;
float d_gain_y = 0.0;

int max_yaw = 300;
int max_roll = 300;
int max_pitch = 300;

signed int g[3]={};                     
int counter, calquant, regime;
const int debug = 0;

float throttle;
float looptime;
float sys_error, last_r, last_p, last_y;
float output_r, output_p, output_y;
float integral_r, integral_p, integral_y;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
double setpoint_r, setpoint_p, setpoint_y;
double input_r, input_p, input_y;
double esc_1, esc_2, esc_3, esc_4;

 
PwmOut esc1(PTA2);
PwmOut esc2(PTC2);
PwmOut esc3(PTA1);
PwmOut esc4(PTC3);

PwmIn ch1(PTD0);
PwmIn ch2(PTD2);
PwmIn ch3(PTD3);
PwmIn ch4(PTD1);

Timer t;

//const int gyro_addr = 0x69 << 1;
//I2C gyro(PTE25, PTE24); // SDA, SCL

L3G4200D gyro(PTE25, PTE24);
Serial pc(USBTX, USBRX); // tx, rx


void pid_controller(){
 // ################ ROLL
    sys_error = input_r - setpoint_r;
    integral_r += (i_gain_r*sys_error);
    if(integral_r > max_roll){
        integral_r = max_roll;
    }
    else if(integral_r < max_roll * -1){
        integral_r = max_roll * -1;
    }
    output_r = -(integral_r + (p_gain_r * sys_error) + (d_gain_r * (sys_error - last_r))); //roll right is positive output
    last_r = sys_error;

    // ################ PITCH
    sys_error = input_p - setpoint_p;
    integral_p += (i_gain_p*sys_error);
    if(integral_p > max_pitch){
        integral_p = max_pitch;
    }
    else if(integral_p < max_pitch * -1){
        integral_p = max_pitch * -1;
    }
    output_p = -(integral_p + (p_gain_p * sys_error) + (d_gain_p * (sys_error - last_p))); //pitch forward is positive output
    last_p = sys_error;

    // ################ YAW
    sys_error = input_y - setpoint_y;
    integral_y += (i_gain_y*sys_error);
    if(integral_y > max_yaw){
        integral_y = max_yaw;
    }
    else if(integral_y < max_yaw * -1){
        integral_y = max_yaw * -1;
    }
    output_y = -(integral_y + (p_gain_y * sys_error) + (d_gain_y * (sys_error - last_y))); //yaw right is positive output
    last_y = sys_error;

    // ################ BOUNDARY CONDITIONING
    if(output_r > max_roll)output_r = max_roll;
    else if(output_r < max_roll * -1)output_r = max_roll * -1;
    if(output_p > max_pitch)output_p = max_pitch;
    else if(output_p < max_pitch * -1)output_p = max_pitch * -1;
    if(output_y > max_yaw)output_y = max_yaw;
    else if(output_y < max_yaw * -1)output_y = max_yaw * -1;

}


void read_gyro(){
    gyro.read(g);
    gyro_roll =     (double) g[0];// CCW -, CW + pointing along X
    gyro_pitch =    (double) g[1]; // CCW -, CW + pointing along Y
    gyro_yaw =  -1* (double) g[2];   // CCW +, CW -

    if(calquant == 2000)gyro_roll -= gyro_roll_cal; 
    if(calquant == 2000)gyro_pitch -= gyro_pitch_cal; 
    if(calquant == 2000)gyro_yaw -= gyro_yaw_cal; 
}


void setup_procedure(){
    esc1.period(0.02);
    esc2.period(0.02);
    esc3.period(0.02);
    esc4.period(0.02);
    esc1.pulsewidth_us(1000);
    esc2.pulsewidth_us(1000);
    esc3.pulsewidth_us(1000);
    esc4.pulsewidth_us(1000);
    wait(5);
    
    if(debug){pc.printf("Motor Initialization Complete \r\n");}
   
    for (calquant = 0; calquant < 2000 ; calquant ++){              
        read_gyro();
        gyro_roll_cal += gyro_roll;
        gyro_pitch_cal += gyro_pitch;
        gyro_yaw_cal += gyro_yaw;
        wait_us(5);
    }
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
    gyro_yaw_cal +=50;


    if(debug){pc.printf("Gyro Calibration Complete \r\n");}
    
    // while the receiver is off, throttle is not at bottom, 
    while(ch3.pulsewidth() < 1000 || ch3.pulsewidth() > 1020 || ch4.pulsewidth() < 1400){;}
    
    if(debug){pc.printf("Receiver connected and ready \r\n");}
    
    regime = 0;
    counter = 0;
    t.start();
    looptime = t.read_us();
}


void regime_determination(){
    if(ch3.pulsewidth() < 1020 && ch4.pulsewidth() <1020){
        regime = 1; //in enabled position
    }

    if(regime == 1 && ch3.pulsewidth() < 1020 && ch4.pulsewidth() >1420){
        regime =2; //fly mode because went form enabled position to center position
        integral_r = 0; integral_p = 0; integral_y = 0; 
        last_r = 0; last_p = 0; last_y = 0;
    }

    if(regime ==2 && ch3.pulsewidth() < 1020 && ch4.pulsewidth() > 1970){
        regime = 0; //disable 
    }

}


void main_loop(){

    read_gyro();
    input_r = (input_r * 0.7) + ((gyro_roll / 57.14286) * 0.3);
    input_p = (input_p * 0.7) + ((gyro_pitch / 57.14286) * 0.3);
    input_y = (input_y * 0.7) + ((gyro_yaw / 57.14286) * 0.3);

    regime_determination();

    setpoint_r = 0;
    setpoint_p = 0;
    setpoint_y = 0;

    // MAPPING THE INPUT TO THE DEG/S PARAMETER THE PID CONTROLLER NEEDS
    if(ch1.pulsewidth() > 1488){
        setpoint_r = (ch1.pulsewidth() - 1488)/3.0;
    }
    else if(ch1.pulsewidth() < 1472){
        setpoint_r = (ch1.pulsewidth() - 1472)/3.0;
    }

    if(ch2.pulsewidth() > 1518){
        setpoint_p = (ch2.pulsewidth() - 1518)/3.0;
    }
    else if(ch2.pulsewidth() < 1498){
        setpoint_p = (ch2.pulsewidth() - 1498)/3.0;
    }

    if(ch3.pulsewidth() > 1020){
        if(ch4.pulsewidth() > 1508){
            setpoint_y = (ch4.pulsewidth() - 1508)/3.0;
        }
        else if(ch4.pulsewidth() < 1492){
            setpoint_y = (ch4.pulsewidth() - 1492)/3.0;
        }
    }

    pid_controller();
    throttle = ch3.pulsewidth();
    
    if (regime == 2){ //fly mode
    
        esc_1 =  throttle - output_p - output_r + output_y;
        esc_2 =  throttle + output_p - output_r - output_y;
        esc_3 =  throttle + output_p + output_r + output_y;
        esc_4 =  throttle - output_p + output_r - output_y;
        
        /*

          \ (2)CW   / (1) CCW
           \       /
            \     /
             \   /
              \ /
               x
              / \
             /   \
            /     \
           /       \
          / (3)CCW  \ (4) CW

        */

        //BOUNDARY CONDITIONING
        if (esc_1 < 1100) esc_1 = 1100;
        if (esc_2 < 1100) esc_2 = 1100;
        if (esc_3 < 1100) esc_3 = 1100;
        if (esc_4 < 1100) esc_4 = 1100;
        if(esc_1 > 2000) esc_1 = 2000;
        if(esc_2 > 2000) esc_2 = 2000;
        if(esc_3 > 2000) esc_3 = 2000;
        if(esc_4 > 2000) esc_4 = 2000;
        
    }
    else{ 
        esc_1 = 500;  //default values
        esc_2 = 500;
        esc_3 = 500;
        esc_4 = 500;
    }
    esc1.pulsewidth_us(esc_1);
    esc2.pulsewidth_us(esc_2);
    esc3.pulsewidth_us(esc_3);
    esc4.pulsewidth_us(esc_4);
  
    if(counter == 500 && debug){
        pc.printf("gyro_roll: %f , gyro_yaw: %f , gyro_pitch: %f \r\n", gyro_roll, gyro_yaw, gyro_pitch);
        pc.printf("output_r: %f , output_p: %f , output_y: %f \r\n",output_r, output_p, output_y);
        pc.printf("ch1: %f , ch2: %f , ch3: %f , ch4: %f \r\n", ch1.pulsewidth(), ch2.pulsewidth(), ch3.pulsewidth(), ch4.pulsewidth());
        pc.printf("esc1: %f , esc2: %f , esc3: %f , esc4: %f \r\n", esc_1, esc_2, esc_3, esc_4);
        pc.printf("Regime: %d,\r\n", regime);
        pc.printf("looptime: %f \r\n\r\n", t.read_us()-looptime);
        counter = 0;
    }

    while(t.read_us() - looptime < 4000);
    looptime = t.read_us();
    counter ++;
}

 

 
int main(){ 
    setup_procedure(); 
    while(1){  
        main_loop();
    }
}