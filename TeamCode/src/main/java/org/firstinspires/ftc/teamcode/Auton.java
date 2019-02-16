package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auton extends OpMode {
    DcMotor motorRightA;                    // creates motors in code
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor spinner;
    Servo liftRight;
    Servo liftLeft;
    Servo grabberRight;
    Servo grabberLeft;
    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        liftRight = hardwareMap.servo.get("liftR");
        liftLeft = hardwareMap.servo.get("liftL");
        grabberRight = hardwareMap.servo.get("grabberR");
        grabberLeft = hardwareMap.servo.get("grabberL");
        spinner = hardwareMap.dcMotor.get("spinner");
        //motorRightA.setDirection(DcMotor.Direction.REVERSE);      //think about logic of motors and how you need to reverse two of them
        //motorRightB.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop(){
        motorLeftA.setPower(1);
    }
}
