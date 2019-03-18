package org.firstinspires.ftc.teamcode;

class Drivetrain {
    EncodedDcMotor mRA;
    EncodedDcMotor mRB;
    EncodedDcMotor mLA;
    EncodedDcMotor mLB;
    Drivetrain(EncodedDcMotor m1, EncodedDcMotor m2, EncodedDcMotor m3, EncodedDcMotor m4){
        mRA = m1;
        mRB = m2;
        mLA = m3;
        mLB = m4;
    }
    void moveY(double distance, double power){
        mRA.travelDistance(distance, power);
        mRB.travelDistance(distance, power);
        mLA.travelDistance(-distance, power);
        mLB.travelDistance(-distance, power);
    }
    void moveX(double distance, double power){
        mRA.travelDistance(distance, power);
        mRB.travelDistance(-distance, power);
        mLA.travelDistance(distance, power);
        mLB.travelDistance(-distance, power);
    }
    void rotate(double deg){
        //Needs implementation
    }
}
