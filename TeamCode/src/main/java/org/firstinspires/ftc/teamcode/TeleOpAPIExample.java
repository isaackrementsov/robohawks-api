package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class TeleOpAPIExample extends OpMode {

    Robot bot;
    double speed = 0.25;

    public void init(){
        this.bot = new Robot(hardwareMap);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);
    }

    public void loop() {
        /*double rightX = gamepad1.right_stick_x; //Strafe
        double rightY = gamepad1.right_stick_y; //Speed
        double leftX = gamepad1.left_stick_x; //Yaw (Turn)

        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);// sets a value check to make sure we don't go over the desired speed (related to joysticks)
        leftX = Range.clip(leftX, -1, 1);

        // Speed adjustment

        if (Math.abs(rightX) > .1 || Math.abs(leftX) > .1 || Math.abs(rightY) > .1) { //Remember that left motor directions are reversed
            bot.dcMotors.get("mRF").setPower((rightY + rightX + leftX) * speed);
            bot.dcMotors.get("mRB").setPower((rightY - rightX + leftX)  * speed);
            bot.dcMotors.get("mLF").setPower((-rightY + rightX + leftX) * speed); //Opposite (negative) of right because the motors are opposite
            bot.dcMotors.get("mLB").setPower((-rightY - rightX + leftX) * speed);
        } else { //will not move if joysticks are not moving
            bot.stop();
        }*/

        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        leftX = Range.clip(leftX, -1, 1);
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);

        if(gamepad1.dpad_up && speed < 4) speed += 0.1;
        if(gamepad1.dpad_down && speed > 1) speed -= 0.1;

        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1){
            bot.drive(-speed, leftX, rightX, rightY);
        }else{
            bot.stop();
        }
    }

}
