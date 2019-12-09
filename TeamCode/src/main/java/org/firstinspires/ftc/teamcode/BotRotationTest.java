package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous
public class BotRotationTest extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{32, 32, 32, 32},
                new double[]{560, 560, 560, 560},
                new double[]{10.5, 10.5, 10.5, 10.5},
                true
        );

        waitForStart();

        bot.rotate(0.2, 30.0);
    }
}
