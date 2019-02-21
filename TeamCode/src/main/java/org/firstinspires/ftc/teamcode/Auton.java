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
    private DcMotor motorRightA;                    // creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter;//1 rev = 2 inches, 8.5 inch range, will start at top
    private DcMotor spinner;
    private DcMotor inOut;
    @Override
    public void runOpMode(){
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        inOut = hardwareMap.dcMotor.get("inOut");
        lifter = hardwareMap.dcMotor.get("lifter");
        spinner = hardwareMap.dcMotor.get("spinner");
        motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Autonomous to move robot forward and lift lifter
        lifter.setTargetPosition(-LIFTER_TICK_COUNTS);
        while(lifter.isBusy());
        lifter.setTargetPosition(LIFTER_TICK_COUNTS);
        while (lifter.isBusy());
        motorRightA.setTargetPosition(DCMOTOR_TICK_COUNTS);
        motorRightB.setTargetPosition(DCMOTOR_TICK_COUNTS);
        motorLeftA.setTargetPosition(-DCMOTOR_TICK_COUNTS); //Left is reversed
        motorLeftB.setTargetPosition(-DCMOTOR_TICK_COUNTS);

    }
}
