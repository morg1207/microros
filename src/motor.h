#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    int pin_pwm_;    // Pin para dirección
    int pin_dir_; // Pin para habilitar/deshabilitar el motor
    float pwm_speed_; // Velocidad actual del motor (0-100%)
    int channel_;

public:
    // Constructor
    Motor(int pin_pwm, int pin_dir, int channel);

    void stop();
    void init();
    void reset();
    
    // Métodos públicos
    void setPwm(float speed); // Setea un pwm 
};

#endif // MOTOR_H
