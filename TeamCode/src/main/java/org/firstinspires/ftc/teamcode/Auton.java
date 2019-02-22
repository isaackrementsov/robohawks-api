package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auton extends LinearOpMode {
    private static final int LIFTER_TICK_COUNTS = 1120; //Tick counts for encoded motors
    private static final int DCMOTOR_TICK_COUNTS = 288;
    private DcMotor motorRightA;   //323.6mm per rev                 creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter;//1 rev = 2 inches, 8.5 inch range, will start at top
    private DcMotor spinner;
    private DcMotor inOut;
    private Servo grabberRight;
    private Servo grabberLeft;
    private Servo locker;
    @Override
    public void runOpMode() throws  InterruptedException {
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        inOut = hardwareMap.dcMotor.get("inOut");
        lifter = hardwareMap.dcMotor.get("lifter");
        spinner = hardwareMap.dcMotor.get("spinner");
        grabberRight = hardwareMap.servo.get("grabberR");
        grabberLeft = hardwareMap.servo.get("grabberL");
        locker = hardwareMap.servo.get("locker");
        motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Autonomous to move robot forward and lift lifter
        grabberRight.setPosition(0);
        grabberLeft.setPosition(1);
        while(grabberRight.getPosition() > 0.1);
        locker.setPosition(0.46);
        while(locker.getPosition() < 0.45);
        lifter.setPower(-0.09);
        Thread.sleep(1000);
        lifter.setTargetPosition((int) Math.floor(LIFTER_TICK_COUNTS));
        while (lifter.isBusy());
        motorRightA.setTargetPosition(DCMOTOR_TICK_COUNTS*5);
        motorRightB.setTargetPosition(DCMOTOR_TICK_COUNTS*5);
        motorLeftA.setTargetPosition(-DCMOTOR_TICK_COUNTS*5); //Left is reversed
        motorLeftB.setTargetPosition(-DCMOTOR_TICK_COUNTS*5);
        while(motorLeftA.isBusy());
        spinner.setPower(-1);
        Thread.sleep(100);
        spinner.setPower(0);

    }
}
