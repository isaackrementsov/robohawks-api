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

        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{32,32,32,32},
                new double[]{560,560,560,560},
                true
        );
        bot.addServo("rdR", 180, 180, 0);

        bot.resetServo("rdR");
        waitForStart();

        /*bot.drive(0.3, 30, Robot.Direction.FORWARD);
        bot.drive(0.3, 30, Robot.Direction.BACKWARD);
        bot.drive(0.3,30, Robot.Direction.LEFT);
        bot.drive(0.3, 30, Robot.Direction.RIGHT);*/

        bot.rotateServo("rdR", 180);
    }

}