// TODO: CREATE MORE OPMODES FOR EACH TEAM STARTING POS!!!
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous
public class MoveFoundationAndPark extends LinearOpMode {

    private Robot bot;

    private final double TILE_SIZE = 60.96;

    private final double Y_TO_FOUNDATION = 3.3*TILE_SIZE;

    private double LOW_POWER = 0.2;
    private double HIGH_POWER = 1;

    private Robot.Direction CENTER_DIRECTION = Robot.Direction.LEFT;

    @Override
    public void runOpMode(){
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{31.42, 31.42, 31.42, 31.42},
                new double[]{767.2, 767.2, 767.2, 767.2},
                1.54,
                true
        );

        bot.addServo("intakeFold", 180, 180, 0);
        bot.addServo("bumper", 180, 115, 60);

        bot.resetServo("bumper", 0);

        bot.addLimitTrigger(new String[]{"bumperLimit1", "bumperLimit2"}, new Robot.Action(){

            public void run(Robot bot){
                bot.rotateServo("bumper", 115, 0);
            }

        }, "bumperDeploy");

        waitForStart();

        driveToFoundation();
        moveFoundation();
        park();
    }

    // TODO: Make sure this is the right distance
    private void park(){
        bot.drive(HIGH_POWER, TILE_SIZE*1.5, Robot.Direction.BACKWARD);
    }

    // TODO: Test distances here
    private void moveFoundation(){
        bot.driveUntilEvenTriggered(0.2, Robot.Direction.BACKWARD, "bumperDeploy", Robot.LimitBehavior.AND);

        bot.rotate(HIGH_POWER, 50);
        bot.drive(LOW_POWER, 10, Robot.Direction.LEFT);
        bot.rotate(HIGH_POWER, -30);

        bot.rotateServo("bumper", 65, 0);
    }

    private void driveToFoundation(){
        bot.drive(LOW_POWER, TILE_SIZE, CENTER_DIRECTION);

        bot.rotateServo("intakeFold", 180, 0);

        bot.drive(HIGH_POWER, Y_TO_FOUNDATION, Robot.Direction.FORWARD);
        bot.stop();

        bot.rotate(HIGH_POWER, 90.0);
    }

}
