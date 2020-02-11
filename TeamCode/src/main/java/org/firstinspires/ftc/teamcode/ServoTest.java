package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {

    private Robot bot;


    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);
        //Servo lock = hardwareMap.servo.get("lock");
        bot.addServo("lock", 180, 90, 0);

        waitForStart();
        bot.holdServo("lock", 90.0, 3000);
        bot.resetServo("lock", 3000);
        //telemetry.update();
    }

    /*
    private void timeout(int millis){
        try {
            Thread.sleep(5000);
        }catch(InterruptedException e){}
    }
    */

}