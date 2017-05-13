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

char last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, regime;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
char highByte, lowByte;

float sys_error, last_r, last_p, last_y;
float integral_r, integral_p, integral_y;
float setpoint_r, setpoint_p, setpoint_y;
float input_r, input_p, input_y;

 
PwmOut esc1(PTC11);
PwmOut esc2(PTC10);
PwmOut esc3(PTA2);
PwmOut esc4(PTC2);

//I2C gyro(PTE25, PTE24); // SDA, SCL
L3G4200D gyro(PTE25, PTE24);

const int gyro_addr = 0x69 << 1;
char cmd[2];

Serial pc(USBTX, USBRX); // tx, rx
//pc.baud(115200);



//rewrite the i2c parts, pretty much all of this
//void startup_procedure(){
    // set up the i2c stuff
    // set i2c clock speed 
    // declare your PWM outputs
    // declare your LED outputs
    // initialize IMU registers
    // take gyro calibration samples
    // set up all your interrupts
    // wait until receiver is active and throttle is in lower position
    // read the battery voltage...
    //  loop_timer = micros();
//}

// rewrite the timer loop towards the bottom
//void main_loop(){
    // generate your RPY inputs from the receiver and the gyroscope
    // set regime to start when yaw lower left and throttle minimum
    // set refime to start2 when yaw is moved over, set pid vals to zero
    // add if statement for shutting off, throttle low, and yaw right
    // then create the linear relationship from input to degrees per second setpoint
    // calculate your PID
    // if start2 regime, sum up all of your esc PWM quantiies

    // if (start == 2){
    //  if (throttle > 1800) throttle = 1800;//We need some room to keep full control at full throttle.
    //      esc_1 = throttle - output_p + output_r - output_y; //Calculate the pulse for esc 1 (front-right - CCW)
    //      esc_2 = throttle + output_p + output_r + output_y; //Calculate the pulse for esc 2 (rear-right - CW)
    //      esc_3 = throttle + output_p - output_r - output_y; //Calculate the pulse for esc 3 (rear-left - CCW)
    //      esc_4 = throttle - output_p - output_r + output_y; //Calculate the pulse for esc 4 (front-left - CW)

    //      if (esc_1 < 1200) esc_1 = 1200;
    //      if (esc_2 < 1200) esc_2 = 1200;
    //      if (esc_3 < 1200) esc_3 = 1200;
    //      if (esc_4 < 1200) esc_4 = 1200;

    //      if(esc_1 > 2000) esc_1 = 2000;
    //      if(esc_2 > 2000) esc_2 = 2000;
    //      if(esc_3 > 2000) esc_3 = 2000;
    //      if(esc_4 > 2000) esc_4 = 2000;
    // }
    // else{
    //  esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    //  esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    //  esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    //  esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
    // }

    // then wait to see if 4ms have passed
    // reset the timer
    // apply the pulse correctly. use library or ports???

//}

// this basically needs to measure the incoming pulse length...
// ISR(PCINT0_vect){
//   current_time = micros();
  
//   //Channel 1=========================================
//   if(PINB & B00000001){                                        //Is input 8 high?
//     if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
//       last_channel_1 = 1;                                      //Remember current input state
//       timer_1 = current_time;                                  //Set timer_1 to current_time
//     }
//   }
//   else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
//     last_channel_1 = 0;                                        //Remember current input state
//     receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
//   }
//   //Channel 2=========================================
//   if(PINB & B00000010 ){                                       //Is input 9 high?
//     if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
//       last_channel_2 = 1;                                      //Remember current input state
//       timer_2 = current_time;                                  //Set timer_2 to current_time
//     }
//   }
//   else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
//     last_channel_2 = 0;                                        //Remember current input state
//     receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
//   }
//   //Channel 3=========================================
//   if(PINB & B00000100 ){                                       //Is input 10 high?
//     if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
//       last_channel_3 = 1;                                      //Remember current input state
//       timer_3 = current_time;                                  //Set timer_3 to current_time
//     }
//   }
//   else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
//     last_channel_3 = 0;                                        //Remember current input state
//     receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

//   }
  
//   //Channel 4=========================================
//   if(PINB & B00001000 ){                                       //Is input 11 high?
//     if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
//       last_channel_4 = 1;                                      //Remember current input state
//       timer_4 = current_time;                                  //Set timer_4 to current_time
//     }
//   }
//   else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
//     last_channel_4 = 0;                                        //Remember current input state
//     receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
//   }
// }

// rewrite this using the new library. should be the same data however
// void read_gyro(){
//   Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
//   Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
//   Wire.endTransmission();                                      //End the transmission
//   Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
//   while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
//   lowByte = Wire.read();                                       //First received byte is the low part of the angular data
//   highByte = Wire.read();                                      //Second received byte is the high part of the angular data
//   gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
//   if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
//   lowByte = Wire.read();                                       //First received byte is the low part of the angular data
//   highByte = Wire.read();                                      //Second received byte is the high part of the angular data
//   gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
//   gyro_pitch *= -1;                                            //Invert axis
//   if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
//   lowByte = Wire.read();                                       //First received byte is the low part of the angular data
//   highByte = Wire.read();                                      //Second received byte is the high part of the angular data
//   gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
//   gyro_yaw *= -1;                                              //Invert axis
//   if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
// }

// just make this look
// void pid_controller(){
//  // ################ ROLL
//  sys_error = input_r - setpoint_r;
//  integral_r += (i_gain_r*sys_error);
//  if(integral_r > max_roll){
//      integral_r = max_roll;
//  }
//  else if(integral_r < max_roll * -1){
//      integral_r = max_roll * -1;
//  }
//  output_r = integral_r + (p_gain_r * sys_error) + (d_gain_r * (sys_error - last_r));
//  last_r = sys_error;
//  // ####################

//  // ################ PITCH
//  sys_error = input_p - setpoint_p;
//  integral_p += (i_gain_p*sys_error)
//  if(integral_p > max_pitch){
//      integral_p = max_pitch;
//  }
//  else if(integral_p < max_pitch * -1){
//      integral_p = max_pitch * -1;
//  }
//  output_p = integral_p + (p_gain_p * sys_error) + (d_gain_p * (sys_error - last_p));
//  last_p = sys_error;
//  // ####################

//  // ################ YAW
//  sys_error = input_y - setpoint_y;
//  integral_y += (i_gain_y*sys_error);
//  if(integral_y > max_yaw){
//      integral_y = max_yaw;
//  }
//  else if(integral_y < max_yaw * -1){
//      integral_y = max_yaw * -1;
//  }
//  output_y = integral_y + (p_gain_y * sys_error) + (d_gain_y * (sys_error - last_y));
//  last_y = sys_error;
//  // ####################

//  // ################ BOUNDARY CONDITIONING
//  if(output_r > max_roll)output_r = max_roll;
//  else if(output_r < max_roll * -1)output_r = max_roll * -1;
//  if(output_p > max_pitch)output_p = max_pitch;
//  else if(output_p < max_pitch * -1)output_p = max_pitch * -1;
//  if(output_y > max_yaw)output_y = max_yaw;
//  else if(output_y < max_yaw * -1)output_y = max_yaw * -1;

// }

void read_gyro(){
    signed int g[3]={};
    gyro.read(g);
    gyro_roll = (double) g[0];// CCW -, CW + pointing along X
    gyro_pitch = (double)g[1]; // CCW -, CW + pointing along Y
    gyro_yaw = (double)g[2];   // CCW +, CW -
}
//
//void setup_procedure(){
//    esc1.period(0.02);          // servo requires a 20ms period
//    esc2.period(0.02);          // servo requires a 20ms period
//    esc3.period(0.02);          // servo requires a 20ms period
//    esc4.period(0.02);          // servo requires a 20ms period
//    esc1.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
//    esc2.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
//    esc3.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
//    esc4.pulsewidth_us(800);    // set the initial pulsewidth to 800 us just to keep them on
//    wait(20);
//    
//    cmd[0] = 0x20;
//    cmd[1] = 0x00;
//    gyro.write(gyro_addr, cmd, 2);
//    cmd[0] = 0x0f;
//    gyro.write(gyro_addr, cmd, 2);
//    
//    cmd[0] = 0x23;
//    gyro.write(gyro_addr, cmd, 2);
//    cmd[0] = 0x90;
//    gyro.write(gyro_addr, cmd, 2);
//    wait_ms(300);
//
//    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              
//        read_gyro();
//        gyro_roll_cal += gyro_roll;
//        gyro_pitch_cal += gyro_pitch;
//        gyro_yaw_cal += gyro_yaw;
//        wait_us(3);
//  }
//  gyro_roll_cal /= 2000;
//  gyro_pitch_cal /= 2000;
//  gyro_yaw_cal /= 2000;
  
  //********************************
//  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
//  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
//  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
//  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
//  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
//
//  //Wait until the receiver is active and the throtle is set to the lower position.
//  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
//    start ++;                                                  //While waiting increment start whith every loop.
//    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
//    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
//    delayMicroseconds(1000);                                   //Wait 1000us.
//    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
//    delay(3);                                                  //Wait 3 milliseconds before the next loop.
//    if(start == 125){                                          //Every 125 loops (500ms).
//      digitalWrite(12, !digitalRead(12));                      //Change the led status.
//      start = 0;                                               //Start again at 0.
//    }
//  }
//  start = 0;                                                   //Set start back to 0.
  //***************************************
  
 }
 

 
int main() {    
    //cmd[0]= 0x20;
//    cmd[1]= 0x00;
//    gyro.write(gyro_addr, cmd, 2);
//    cmd[0]= 0x0F;
//    gyro.write(gyro_addr, cmd, 2);
//
//    cmd[0]= 0x23;
//    gyro.write(gyro_addr, cmd, 2);
//    cmd[0]= 0x90;
//    gyro.write(gyro_addr, cmd, 2);
//    
//    wait(1);
     //int g[3];
//    gyro.read(g);
    signed int g[3]={};
    while (1) {
        gyro.read(g);
        pc.printf("roll = %d \r\n", g[0]); 
        pc.printf("pitch = %d \r\n", g[1]);
        pc.printf("yaw = %d \r\n", g[2]); 
    }  
}