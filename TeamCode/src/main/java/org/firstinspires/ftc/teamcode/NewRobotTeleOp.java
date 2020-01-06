package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class NewRobotTeleOp extends OpMode {

    private Robot bot;

    private double power = 0.2;

    private boolean bumperUp;

    public void init(){
        bumperUp = false;

        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        bot.addDcMotor("intake1");
        bot.addDcMotor("intake2");

        bot.addLimitedMotor("inOut", "limitOut", "limitIn");
        bot.addLimitedMotor("upDown", new String[]{"limitDown"}, new String[]{"limitUp1", "limitUp2"});

        bot.addServo("bumper", 180, 90, 0);
        bot.addServo("gripperTurn", 0);
        bot.addServo("gripper", 180, 180, 0);

        bot.resetServo("gripperTurn", 0);
    }

    public void loop(){
        processController1();
        processController2();
    }

    private void processController1(){
        // Driver controls
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        // Keep joysticks between -1 and 1
        leftX = Range.clip(leftX, -1, 1);
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);

        // Set different power modes
        if(gamepad1.dpad_up) power = 2;
        if(gamepad1.dpad_right) power = 1;
        if(gamepad1.dpad_left) power = 0.2;
        if(gamepad1.dpad_down) power = 0.1;

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1){
            bot.drive(power, leftX, rightX, rightY);
        }else{
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        // Spin the two intake motors
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;

        // Keep triggers between 0 and 1
        triggerRight = Range.clip(triggerRight, 0, 1);
        triggerLeft = Range.clip(triggerLeft, 0, 1);

        double intakePower = 0;

        // Set the intake power based on triggers if they are pressed
        if(triggerRight > .1) intakePower = 4;
        else if(triggerLeft > .1) intakePower = -4;

        bot.moveDcMotor("intake1", intakePower);
        bot.moveDcMotor("intake2", -intakePower);

        // Run the foundation grabber
        boolean bumperRight = gamepad1.right_bumper;
        boolean bumperLeft = gamepad1.left_bumper;

        double pos = 0;

        if(bumperLeft) {
            bumperUp = false;
            pos = 0;
        }else if(bumperRight || bumperUp){
            bumperUp = true;
            pos = 89;
        }

        bot.rotateServo("bumper", pos, 0);
    }

    private void processController2(){
        // Arm motor in-out, controlled by right joystick Y
        double rightY = gamepad2.right_stick_y;

        rightY = Range.clip(rightY, -1, 1);

        // Make sure joy is being moved
        if(Math.abs(rightY) > .1){
            bot.moveLimitedMotor("inOut", rightY);
        }else{
            bot.moveDcMotor("inOut", 0);
        }

        // Rotate gripper servo, controlled by left joystick X (precise) and dpad (general)
        double leftX = gamepad2.left_stick_x;
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        leftX = Range.clip(leftX, -1, 1);

        telemetry.addData("Dpad right", dpadRight);
        telemetry.addData("dpad left", dpadLeft);

        // Use dpads to move rigidly between 0 and 180
        if(dpadRight){
            bot.rotateServo("gripperTurn", 179.9, 0);
        }else if(dpadLeft){
            bot.rotateServo("gripperTurn", 0.1, 0);
        }/*else if(Math.abs(leftX) > .1){ // Operate precise controls if joy is pressed
            bot.incrementServo("gripperTurn", 0.5*leftX, 0);
        }*/

        // Move the arm up and down with triggers
        double triggerRight = gamepad2.right_trigger;
        double triggerLeft = gamepad2.left_trigger;

        triggerRight = Range.clip(triggerRight, 0, 1);
        triggerLeft = Range.clip(triggerLeft, 0, 1);

        // If triggers are pressed, move up/down with limit switches
        if(triggerRight > .1){
            bot.moveLimitedMotorArray("upDown", triggerRight, Robot.LimitBehavior.AND);
        }else if(triggerLeft > .1){
            bot.moveLimitedMotorArray("upDown", -triggerLeft, Robot.LimitBehavior.OR);
        }else{
            bot.moveDcMotor("upDown", 0);
        }

        // Close/Open grabber motor using bumpers
        boolean bumperR = gamepad2.right_bumper;
        boolean bumperL = gamepad2.left_bumper;

        if(bumperR){
            bot.rotateServo("gripper", 180, 0);
        }else if(bumperL){
            bot.rotateServo("gripper", 0, 0);
        }
    }

}
