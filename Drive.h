#ifndef Drive_h
#define Drive_H

// #include <Arduino.h>

#include <Servo.h>

class Drive {
  private:
    int in1;
    int in2;
    int enA;
    int in3;
    int in4;
    int enB;
    int encoder; 
    float rps;
    int rots; 
    int lastEncoderValue;
    Servo steering;
    
    float drive_matrix[3][3];

    float toRadians(float theta);
    void foward();
    void turnRight();
    void onlyRight();
    void onlyLeft();
    void backward();
    void turnLeft();
    void stop();

    // We can have Vy and Vx in any form we want, even rotations per second.
  public:
    Drive(int in1,int in2,int enA,int in3,int in4,int enB,int encoder, int servopin);
    float speed();
    void updateDrive(float dt);

    void set_drive_motors();

    bool change_in_encoder();

    /**
     * @brief Set the Drive object
     * 
     * @param pctpower 
     * @param dir 
            -1: stop()
             0: foward()
             1: onlyLeft()
             2: turnLeft()
             3: onlyRight()
             4: turnRight()
             5: backward()
     * @param theta 
     */
    void setDrive(float pctpower,int dir, float theta);

};
#endif