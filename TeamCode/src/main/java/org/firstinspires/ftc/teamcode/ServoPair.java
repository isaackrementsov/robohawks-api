package org.firstinspires.ftc.teamcode;

class ServoPair {
    ServoX servo1;
    ServoX servo2;
    ServoPair(ServoX s1, ServoX s2){
        servo1 = s1;
        servo2 = s2;
    }
    void setPosition(int angle){
        servo1.setPosition(angle);
        servo2.setPosition(servo1.totalAngle - angle);
    }
}
