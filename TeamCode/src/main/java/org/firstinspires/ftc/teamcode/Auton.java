package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous


public class Auton extends LinearOpMode {

    private static final int LIFTER_TICK_COUNTS = 1120; //Tick counts for encoded motors
    private static final int INOUT_TICK_COUNTS = 4; //Tick counts for encoded motors
    private static final int GRABBER_TICK_COUNTS = 28;
    private static final int DCMOTOR_TICK_COUNTS = 288;
    private DcMotor motorRightA;   //323.6mm per rev                 creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter;//1 rev = 2 inches, 8.5 inch range, will start at top
    private DcMotor spinner;
    private DcMotor inOut;
    private DcMotor grabber;
    //private Servo grabberRight;
    //private Servo grabberLeft;
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
        //grabberRight = hardwareMap.servo.get("grabberR");
        //grabberLeft = hardwareMap.servo.get("grabberL");
        grabber = hardwareMap.dcMotor.get("grabber");
        locker = hardwareMap.servo.get("locker");
        motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        // Autonomous to move robot forward and lift lifter
        //grabberRight.setPosition(0);
        //grabberLeft.setPosition(1);
        //grabber.setTargetPosition((int) -Math.floor(GRABBER_TICK_COUNTS*0.56));
        //grabber.setTargetPosition(100);
        //grabber.setPower(-0.5);
        //while(grabber.isBusy());
        grabber.setPower(-0.51);
        sleep(375);
        grabber.setPower(-0.25);
        sleep(600);
        grabber.setPower(0);
        inOut.setTargetPosition(-INOUT_TICK_COUNTS*2);
        inOut.setPower(0.3);
        locker.setPosition(0.36); //sets locker to position to allow robot to drop to the ground
        while(locker.getPosition() < 0.35);
        lifter.setPower(-0.14); //lets robot down *gently*
       // Thread.sleep(2000); //hopefully on ground
        //lifter.setPower(0);
        lifter.setTargetPosition((int) Math.floor(LIFTER_TICK_COUNTS*3.56));


     //   Thread.sleep(1);
    //    while (lifter.isBusy());
        motorRightA.setTargetPosition(DCMOTOR_TICK_COUNTS*5);
        motorRightB.setTargetPosition(DCMOTOR_TICK_COUNTS*5);
        motorLeftA.setTargetPosition(-DCMOTOR_TICK_COUNTS*5); //Left is reversed
        motorLeftB.setTargetPosition(-DCMOTOR_TICK_COUNTS*5);
        while(motorLeftA.isBusy());
        spinner.setPower(-1);
        sleep(100);
        spinner.setPower(0);

    }
}
