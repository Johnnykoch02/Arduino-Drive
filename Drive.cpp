#include "Arduino.h"
#include "Drive.h"
// #include <iostream>

// const double PI = 3.141593;

    // int in1;
    // int in2;
    // int enA;
    // int in3;
    // int in4;
    // int enB;
    // int encoder; 
    // float rps;
    float driveDtTotal = 0;
    int rots = 0;   
    int driveLoopCount = 0;
    int lastEncoderValue = 0;
    int setDriveCnt = 0;
    float drive_matrix[3][3] = {{0, 90, 0}, // Current Counts, THETA, X COORD
                               {0, 0, 0}, // Velocity Y, Velocity X, Y COORD
                              {0, 0, 0}};  // NULL, DELTA Vy, DELTA Vx
                              // We can have Vy and Vx in any form we want, even rotations per second.

    Drive::Drive(int in1,int in2,int enA,int in3,int in4,int enB,int encoder, int servopin) {
        this->in1=in1;
        this->in2=in2;
        this->enA=enA;
        this->in3=in3;
        this->in4=in4;
        this->enB=enB;
        this->encoder=encoder;
        steering.attach(servopin);

    }

    float Drive::speed() { return this->rps; }
    void Drive::updateDrive(float dt) {
      driveLoopCount++;
      driveDtTotal+=dt;
      //Serial.println(driveDtTotal);

      if (driveLoopCount == 25) {
        //Serial.println("Inside DriveLoop");
        rps = (drive_matrix[0][0]/(1/driveDtTotal))/9;
        drive_matrix[0][0] = 0;
        driveDtTotal=0;
        driveLoopCount = 0;
        drive_matrix[1][0] = sin(toRadians(drive_matrix[0][1])) * rps;
        drive_matrix[1][1] = cos(toRadians(drive_matrix[0][1])) * rps;
        drive_matrix[0][2] += drive_matrix[1][1] * dt;
        drive_matrix[1][2] += drive_matrix[1][0] * dt;
      }


       // Wonder how to get acceleration...? 
      
      if (change_in_encoder()) {
        drive_matrix[0][0]++;
      }
    }

    void Drive::set_drive_motors() {
      pinMode(enA, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(enB, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
      pinMode(encoder, INPUT);
    }

    bool Drive::change_in_encoder() {
      int i = digitalRead(encoder);
      if (i != lastEncoderValue) {
        lastEncoderValue = i;
        return true;
      }
      return false;
    }


    void Drive::setDrive(float pctpower, int dir, float theta) {
        setDriveCnt++;
        //Please change this 
        if(setDriveCnt == 5) {
            setDriveCnt = 0;
            // Serial.println(dir);
            analogWrite(enB, 255 * (pctpower / 100));
            analogWrite(enA, 255 * (pctpower / 100));
            if (dir == -1) { stop(); }
            else if (dir == 0) { foward(); }
            else if (dir == 1) { onlyLeft(); }
            else if (dir == 2) { turnLeft(); } //turn hard left
            else if (dir == 3) { onlyRight(); } // turn right
            else if (dir == 4) { turnRight(); } // turn hard right
            else if (dir == 5) { backward(); }
        }
     
        
    }


    float Drive::toRadians(float theta) {
        return theta * PI / 180;
    }

    void Drive::foward() {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }    
    void Drive::stop() {
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    void Drive::turnLeft() {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

    } 

    void Drive::onlyLeft() {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

    }       
    
    void Drive::turnRight() {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }

    void Drive::onlyRight() {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    void Drive::backward() {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
