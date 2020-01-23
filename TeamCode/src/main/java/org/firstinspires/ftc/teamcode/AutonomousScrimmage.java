package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous
public class AutonomousScrimmage extends LinearOpMode {

    private Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        waitForStart();

        bot.drive(0.2, Robot.Direction.FORWARD);

        Thread.sleep(1000);

        bot.drive(0.2, Robot.Direction.RIGHT);

        Thread.sleep(3000);

        bot.stop();
    }
}
