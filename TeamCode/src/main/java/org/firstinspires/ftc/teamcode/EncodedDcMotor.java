package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

class EncodedDcMotor {
    private DcMotor motor;
    private int ticks;
    private  int circumference;
    private int gearingRatio;
    EncodedDcMotor(DcMotor m, int t, int circ, int gR){
        motor = m;
        ticks = t;
        circumference = circ;
        gearingRatio = gR;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void travelDistance(double distance){
        int pos = (int) Math.round(gearingRatio*ticks*distance/circumference);
        motor.setTargetPosition(pos);
    }
}
