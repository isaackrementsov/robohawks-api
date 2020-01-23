package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class DrivetrainMotorTest extends OpMode {

    private Robot bot;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDcMotors(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{32, 32, 32, 32},
                new double[]{560, 560, 560, 560},
                true
        );
    }

    public void loop(){
        if(gamepad1.x){
            bot.moveDcMotor("mRF", 0.2);
        }else if(gamepad1.y){
            bot.moveDcMotor("mLF", 0.2);
        }else if(gamepad1.a){
            bot.moveDcMotor("mRB", 0.2);
        }else if(gamepad1.b){
            bot.moveDcMotor("mLB", 0.2);
        }else{
            bot.stop();
        }
    }

}
