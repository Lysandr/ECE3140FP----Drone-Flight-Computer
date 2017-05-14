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


float p_gain_r = 1.3;
float i_gain_r = 0;
float d_gain_r = 15;
float p_gain_p = 1.3;
float i_gain_p = 0;
float d_gain_p = 15;
float p_gain_y = 4.0;
float i_gain_y = 0.0;
float d_gain_y = 0.0;
int max_yaw = 400;
int max_roll = 400;
int max_pitch = 400;
int counter;

char last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
//int throttle;

int calquant, regime;
float loop_timer; // this is important for keeping the loop running at the right speed
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal
;
float sys_error, last_r, last_p, last_y;
float output_r, output_p, output_y;
float integral_r, integral_p, integral_y;
float setpoint_r, setpoint_p, setpoint_y;
float input_r, input_p, input_y;

 
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



// just make this look
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
 output_r = integral_r + (p_gain_r * sys_error) + (d_gain_r * (sys_error - last_r));
 last_r = sys_error;
 // ####################

 // ################ PITCH
 sys_error = input_p - setpoint_p;
 integral_p += (i_gain_p*sys_error);
 if(integral_p > max_pitch){
     integral_p = max_pitch;
 }
 else if(integral_p < max_pitch * -1){
     integral_p = max_pitch * -1;
 }
 output_p = integral_p + (p_gain_p * sys_error) + (d_gain_p * (sys_error - last_p));
 last_p = sys_error;
 // ####################

 // ################ YAW
 sys_error = input_y - setpoint_y;
 integral_y += (i_gain_y*sys_error);
 if(integral_y > max_yaw){
     integral_y = max_yaw;
 }
 else if(integral_y < max_yaw * -1){
     integral_y = max_yaw * -1;
 }
 output_y = integral_y + (p_gain_y * sys_error) + (d_gain_y * (sys_error - last_y));
 last_y = sys_error;
 // ####################

 // ################ BOUNDARY CONDITIONING
 if(output_r > max_roll)output_r = max_roll;
 else if(output_r < max_roll * -1)output_r = max_roll * -1;
 if(output_p > max_pitch)output_p = max_pitch;
 else if(output_p < max_pitch * -1)output_p = max_pitch * -1;
 if(output_y > max_yaw)output_y = max_yaw;
 else if(output_y < max_yaw * -1)output_y = max_yaw * -1;

}





void read_gyro(){
    signed int g[3]={};
    
    gyro.read(g);
    gyro_roll =     (double) g[0];// CCW -, CW + pointing along X
    gyro_pitch =    (double) g[1]; // CCW -, CW + pointing along Y
    gyro_yaw =      (double) g[2];   // CCW +, CW -

    if(calquant == 2000)gyro_roll -= gyro_roll_cal; 
    if(calquant == 2000)gyro_pitch -= gyro_pitch_cal; 
    if(calquant == 2000)gyro_yaw -= gyro_yaw_cal; 
}


void setup_procedure(){
    esc1.period(0.02);          // servo requires a 20ms period
    esc2.period(0.02);          // servo requires a 20ms period
    esc3.period(0.02);          // servo requires a 20ms period
    esc4.period(0.02);          // servo requires a 20ms period
    esc1.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
    esc2.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
    esc3.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
    esc4.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
    wait_s(5);
   
    for (calquant = 0; calquant < 2000 ; calquant ++){              
        read_gyro();
        gyro_roll_cal += gyro_roll;
        gyro_pitch_cal += gyro_pitch;
        gyro_yaw_cal += gyro_yaw;
        wait_us(3);
    }
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;

    // while the receiver is off, or high, or ch 4 is left
    while(ch3.pulsewidth() < 1000 || ch3.pulsewidth() > 1020 || ch4.pulsewidth() < 1400){;}

    regime = 0;
    counter = 0;
    t.start();
    loop_timer = t.read();
}


void main_loop(){

    read_gyro();
    input_r = (input_r * 0.7) + ((gyro_roll / 57.14286) * 0.3);
    input_p = (input_p * 0.7) + ((gyro_pitch / 57.14286) * 0.3);
    input_y = (input_y * 0.7) + ((gyro_yaw / 57.14286) * 0.3);

    

    if(ch3.pulsewidth() < 1020 && ch4.pulsewidth() <1020){
        regime = 1;
    }

    if(regime == 1 && ch3.pulsewidth() < 1020 && ch4.pulsewidth() >1420){
        regime =2;
        integral_r = 0; integral_p = 0; integral_y = 0; 
        last_r = 0; last_p = 0; last_y = 0;
    }

    if(regime ==2 && ch3.pulsewidth() < 1020 && ch4.pulsewidth() > 1970){
        regime = 0;
    }

    setpoint_r = 0;
    setpoint_p = 0;
    setpoint_y = 0;

    if(ch1.pulsewidth() > 1508){
        setpoint_r = (ch1.pulsewidth() - 1508)/3.0;
    }
    else if(ch1.pulsewidth() < 1492){
        setpoint_r = (ch1.pulsewidth() - 1492)/3.0;
    }

    if(ch2.pulsewidth() > 1508){
        setpoint_p = (ch2.pulsewidth() - 1508)/3.0;
    }
    else if(ch2.pulsewidth() < 1492){
        setpoint_p = (ch1.pulsewidth() - 1492)/3.0;
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
    
    if (regime == 2){
        esc_1 =  -output_p + output_r - output_y; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 =  output_p + output_r + output_y; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 =  output_p - output_r - output_y; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 =  -output_p - output_r + output_y; //Calculate the pulse for esc 4 (front-left - CW)

        if (esc_1 < 1200) esc_1 = 1200;
        if (esc_2 < 1200) esc_2 = 1200;
        if (esc_3 < 1200) esc_3 = 1200;
        if (esc_4 < 1200) esc_4 = 1200;

        if(esc_1 > 2000) esc_1 = 2000;
        if(esc_2 > 2000) esc_2 = 2000;
        if(esc_3 > 2000) esc_3 = 2000;
        if(esc_4 > 2000) esc_4 = 2000;  
    }
    else{
        esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
        esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
        esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
        esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
    }

    esc1.pulsewidth_us(esc_1);    // set the initial pulsewidth to 800 us just to keep them on
    esc2.pulsewidth_us(esc_2);    // set the initial pulsewidth to 800 us just to keep them on
    esc3.pulsewidth_us(esc_3);    // set the initial pulsewidth to 800 us just to keep them on
    esc4.pulsewidth_us(esc_4);    // set the initial pulsewidth to 800 us just to keep them on
  
    if(counter == 125){
        printf("esc1: %s , esc2: %s , esc3: %s , esc4: %s \r\n",esc_1 ,esc_2 ,esc_3 ,esc_4);
        printf("gyro_roll: %e , gyro_roll: %e , gyro_roll: %e \r\n",gyro_roll, gyro_yaw, gyro_pitch);
        printf("output_r: %e , output_p: %e , output_y: %e \r\n",output_r, output_p, output_y);
        counter = 0;
    }

    while((t.read()*1000) - loop_timer < 4000);
    loop_timer = t.read()*1000;
    counter ++;
}

 

 
int main() { 
    setup_procedure(); 
    while(1){  
        main_loop();
    }
}
