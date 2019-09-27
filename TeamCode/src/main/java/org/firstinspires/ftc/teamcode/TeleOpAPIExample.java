package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

public class TeleOpAPIExample extends OpMode {

    Robot bot;
    int speed = 1;

    public void init(){
        this.bot = new Robot(hardwareMap);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"});
    }

    public void loop() {
        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);
        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
        double rightY = Range.clip(gamepad1.right_stick_y, -1, 1);

        if(gamepad1.dpad_up && speed < 4) speed += 1;
        if(gamepad1.dpad_down && speed > 1) speed -= 1;

        if(leftX > .1 || rightX > .1 || rightY > .1){
            bot.drive(speed, leftX, rightX, rightY);
        }else{
            bot.stop();
        }
    }

}
