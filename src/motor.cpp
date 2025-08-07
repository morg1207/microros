#include "motor.h"
#include <Arduino.h>

// Constructor
Motor::Motor(int pin_pwm, int pin_dir, int channel) 
    : pin_pwm_(pin_pwm), pin_dir_(pin_dir), channel_(channel){

  pwm_speed_ = 0.0;
}

void Motor::init(){

    const int ledChannel = channel_;    // Canal PWM, puede ser de 0 a 15
    const int frequency = 5000;  // Frecuencia en Hz
    const int resolution = 12;    // Resolución en bits (de 1 a 15)

    ledcSetup(ledChannel, frequency, resolution);
    ledcAttachPin(pin_pwm_, ledChannel);

    pinMode(pin_dir_, OUTPUT);
    //Serial.println("Motor Started");

}

void Motor::reset(){

    pwm_speed_ = 0.0;
}

// Establece la velocidad del motor
void Motor::setPwm(float speed) {
    pwm_speed_ = constrain(speed, -4095.0, 4095.0); // Limitar entre -100% y 100%
    //Serial.print("PWM seteado ");
    //Serial.println(pwm_speed_);

    // Determinar dirección
    if (pwm_speed_ > 0) {
        ledcWrite(channel_, abs(pwm_speed_)); 
        digitalWrite(pin_dir_,HIGH);
    }
    else if(pwm_speed_ == 0){
        ledcWrite(channel_, 0); 
        digitalWrite(pin_dir_,LOW);
    } else {
        ledcWrite(channel_, abs(pwm_speed_)); 
        digitalWrite(pin_dir_,LOW);
    }
}

// Habilita el motor
void Motor::stop() {        
    ledcWrite(channel_, 0); 
}
