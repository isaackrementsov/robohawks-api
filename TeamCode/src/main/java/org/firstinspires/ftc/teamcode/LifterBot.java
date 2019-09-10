package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sam on 11/2/2017. Copied and pasted by Willem. Viewed by Milo.
 * Added to by Isaac (and Wolfie for algorithms) on 2/11/2019.
 */
@Autonomous
//Using inheritance to extend other methods
public class LifterBot extends LinearOpMode {
    public void runOpMode(){
        DcMotor motor = hardwareMap.dcMotor.get("mLF");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        int TICKS = 288; //Encoder ticks

        //Basic auton encoder testing
        motor.setTargetPosition(TICKS*2);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Tells motor to stop at specified position
        motor.setPower(0.2); //How fast, on average, to go to the position
        while(motor.isBusy());

        //API Stuff
        /*EncodedDcMotor mRA = new EncodedDcMotor(hardwareMap.dcMotor.get("mRF"), 538, (int) Math.round(1.03*Math.PI), 1);
        EncodedDcMotor mRB = new EncodedDcMotor(hardwareMap.dcMotor.get("mRB"), 538, (int) Math.round(103*Math.PI), 1);
        EncodedDcMotor mLA = new EncodedDcMotor(hardwareMap.dcMotor.get("mLF"), 538, (int) Math.round(103*Math.PI), 1);
        EncodedDcMotor mLB = new EncodedDcMotor(hardwareMap.dcMotor.get("mLB"), 538, (int) Math.round(103*Math.PI), 1);
        Drivetrain robot = new Drivetrain(mRA, mRB, mLA, mLB);
        robot.moveY(10, 3);
        robot.moveX(5, 3);*/
    }
}