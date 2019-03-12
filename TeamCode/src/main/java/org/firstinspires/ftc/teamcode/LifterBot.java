package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
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
@TeleOp
//Using inheritance to extend other methods
public class LifterBot extends OpMode {
    ServoX lifterL;
    ServoX lifterR;
    ServoPair lifter;
    public void init(){
        lifterL = new ServoX(hardwareMap.servo.get("lifterL"), 20, 160, 180, 1);
        lifterR = new ServoX(hardwareMap.servo.get("lifterR"), 160, 20, 180, 1);
        lifter = new ServoPair(lifterL, lifterR);
    }
    public void loop(){
        double rightY = gamepad1.right_stick_y;
        rightY = Range.clip(rightY, -1, 1);
        if(Math.abs(gamepad1.right_stick_y) > 0.1){
            lifter.setPosition((int) Math.floor(180 * rightY));it 
        }
    }
}