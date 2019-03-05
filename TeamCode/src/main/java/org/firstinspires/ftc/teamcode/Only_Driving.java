package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sam on 11/2/2017. Copied and pasted by Willem. Viewed by Milo.
 * Added to by Isaac (and Wolfie for algorithms) on 2/11/2019.
 */
@TeleOp
//Using inheritance to extend other methods
public class Only_Driving extends OpMode {

    private DcMotor motorRightA;                    // creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;

    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
    }
    double speed = 1.75;
    @Override
    public void loop() {
        // goes into loop after the setup is done  in above void
        // sets up a loop for driving the robot
        if(!gamepad1.atRest()){//Make sure that a button is pressed before looping for performance reasons
            loopController1(); //Handles gamepad1 controls
        }
    }
    //Revised based on https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    private void loopController1(){
        // input joystick values into variables that we can use to control the motors
        double rightX = gamepad1.right_stick_x; //Strafe
        double rightY = gamepad1.right_stick_y; //Speed
        double leftX = gamepad1.left_stick_x; //Yaw (Turn)
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);// sets a value check to make sure we don't go over the desired speed (related to joysticks)
        leftX = Range.clip(leftX, -1, 1);

    /*    // Speed adjustment
        if (gamepad1.dpad_up) speed = 4;
        else if (gamepad1.dpad_right) speed = 3;
        else if (gamepad1.dpad_down) speed = 2;
        else if (gamepad1.dpad_left) speed = 1;
    */
        if (Math.abs(gamepad1.right_stick_x) > .2 || Math.abs(gamepad1.left_stick_x) > .2 || Math.abs(gamepad1.right_stick_y) > .2) { //Remember that left motor directions are reversed
            speed = 1;

            motorRightA.setPower((rightY + rightX + leftX) / 4 * speed);
            motorRightB.setPower((rightY - rightX + leftX) / 4 * speed);
            motorLeftA.setPower((-rightY + rightX + leftX) / 4 * speed); //Opposite (negative) of right because the motors are opposite
            motorLeftB.setPower((-rightY - rightX + leftX) / 4 * speed);

            /*
            To help understand the equation put in positive or negative for the motor speeds and see what the outputs
            Strafing - one side goes out (<- ->), one goes in (-> <-)
              Forward       Turn        Strafe
                -> ->       -> ->       -> <-
                -> ->       <- <-       <- ->
               Think about force vectors created by 45° rotation on meccanum wheels to understand strafing
            */
        } else { //will not move if joysticks are not moving
            motorRightA.setPower(0);
            motorRightB.setPower(0);
            motorLeftA.setPower(0);
            motorLeftB.setPower(0);
        }
        telemetry.addData("Joy1", "Joystick 1:  " + String.format("%.2s", gamepad1.left_stick_y)); // feedback given to the driver phone from the robot phone
        telemetry.addData("Joy2", "Joystick 2:  " + String.format("%.2s", gamepad1.right_stick_y));
    }

}