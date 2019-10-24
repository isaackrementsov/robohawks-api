package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutonAPIExample extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap);

        bot.addServo("lock", 180, 180, -180);

        waitForStart();

        telemetry.addData("servo pos", bot.servos.get("lock").getPosition());
        telemetry.update();

        while(!isStopRequested()){

        }
    }

}