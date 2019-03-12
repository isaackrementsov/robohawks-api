package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

class MotorX {
    int endStop1 = Integer.MIN_VALUE;
    int endStop2 = Integer.MAX_VALUE;
    protected double clip(double original){
        return Range.clip(original, endStop1, endStop2);
    }
    protected void setEndstops(int e1, int e2){
        endStop1 = e1;
        endStop2 = e2;
    }
}
