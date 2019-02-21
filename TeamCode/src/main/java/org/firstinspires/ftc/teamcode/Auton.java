package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auton extends OpMode {
    private DcMotor motorRightA;                    // creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter;
    private DcMotor spinner;
    private DcMotor inOut;
    private Servo bucketRight;
    private Servo bucketLeft;
    private Servo grabberRight;
    private Servo grabberLeft;
    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        inOut = hardwareMap.dcMotor.get("inOut");
        lifter = hardwareMap.dcMotor.get("lifter");
        spinner = hardwareMap.dcMotor.get("spinner");
        bucketRight = hardwareMap.servo.get("bucketR");
        bucketLeft = hardwareMap.servo.get("bucketL");
        grabberRight = hardwareMap.servo.get("grabberR");
        grabberLeft = hardwareMap.servo.get("grabberL");
        inOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRightA.setDirection(DcMotor.Direction.REVERSE);      //think about logic of motors and how you need to reverse two of them
        //motorRightB.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop(){
        // Autonomous to move robot forward and lift lifter
        motorLeftA.setTargetPosition(25);
        motorRightA.setTargetPosition(25);
        motorLeftB.setTargetPosition(25);
        motorRightB.setTargetPosition(25);
        lifter.setTargetPosition(25);
    }
}
