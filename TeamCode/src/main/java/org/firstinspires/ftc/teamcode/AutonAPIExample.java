package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonAPIExample extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, new double[]{32,32,32,32}, new double[]{560,560,560,560});

        waitForStart();

        bot.drive(0.3,30, Robot.Direction.LEFT);
        bot.drive(0.3, 30, Robot.Direction.RIGHT);
        bot.drive(0.3, 30, Robot.Direction.FORWARD);
        bot.drive(0.3, 30, Robot.Direction.BACKWARD);
        //Add loop code
    }

}