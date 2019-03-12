package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
class ServoX extends MotorX {
    Servo servo;
    int totalAngle;
    double gearingRatio;
    ServoX(Servo s, int e1, int e2, int tA, double gR) {
        servo = s;
        totalAngle = tA;
        gearingRatio = gR;
        super.setEndstops(e1, e2);
    }
    void setPosition(int angle){
        double angleDec = clip(angle)/totalAngle;
        servo.setPosition(angleDec);
    }
}
