#include "mbed.h"
 
PwmOut motor1(PTC11);
PwmOut motor2(PTC10);
PwmOut motor3(PTA2);
PwmOut motor4(PTC2);

double offset = 0;
 
int main() {
    motor1.period_ms(20);          // servo requires a 20ms period
    while (1) {
        for(offset=1000; offset<2000; offset+=1) {
            motor1.pulsewidth_us(offset); // servo position determined by a pulsewidth between 1-2ms
            motor2.pulsewidth_us(offset); // servo position determined by a pulsewidth between 1-2ms
            motor3.pulsewidth_us(offset); // servo position determined by a pulsewidth between 1-2ms
            motor4.pulsewidth_us(offset); // servo position determined by a pulsewidth between 1-2ms
            wait(0.25);
        }
    }
}
