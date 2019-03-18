package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        EncodedDcMotor mRA = new EncodedDcMotor(hardwareMap.dcMotor.get("mRA"), 538, (int) Math.round(103*Math.PI), 1);
        EncodedDcMotor mRB = new EncodedDcMotor(hardwareMap.dcMotor.get("mRB"), 538, (int) Math.round(103*Math.PI), 1);
        EncodedDcMotor mLA = new EncodedDcMotor(hardwareMap.dcMotor.get("mLA"), 538, (int) Math.round(103*Math.PI), 1);
        EncodedDcMotor mLB = new EncodedDcMotor(hardwareMap.dcMotor.get("mLB"), 538, (int) Math.round(103*Math.PI), 1);
        Drivetrain robot = new Drivetrain(mRA, mRB, mLA, mLB);
        robot.moveY(90000, 0.5);
        robot.moveX(50000, 0.5);
    }
}