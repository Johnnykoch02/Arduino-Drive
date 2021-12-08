This class was designed to be re-usable for controlling a dual motor speed controller for Arduino.

HOW TO USE:
        Drive(int in1,int in2,int enA,int in3,int in4,int enB,int encoder, int servopin);

            - pass in corresponding pins to each of the pins from the speed controller into the 
                Arduino unit.

        1: stop
        0: foward
        1: onlyLeft
        2: turnLeft
        3: onlyRight
        4: turnRight
        5: backward

            - These following commands correspond to the command numbers inside setDrive
            - You can modify pin order, or the drive code until it works for you. 


Future Updates:

    - Add in servo option for controlling a wheel axle
    - Add in encoder logic and PD Controller
    - Be able to translate given a dx and dy value
