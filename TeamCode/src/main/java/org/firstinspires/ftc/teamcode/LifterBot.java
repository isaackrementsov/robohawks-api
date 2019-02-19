package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sam on 11/2/2017. Copied and pasted by Willem. Viewed by Milo.
 * Added to by Isaac (and Wolfie for algorithms) on 2/11/2019.
 */
@TeleOp
//Using inheritance to extend other methods
public class LifterBot extends OpMode {
    private boolean isDumping = false;
    private boolean isReturning = false;
    private boolean bucketReturning = false;
    private boolean inOutDumping = false;
    private boolean grabbersDumping = false;
    private boolean lifterDumping = false;
    private DcMotor motorRightA;                    // creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter;
    private DcMotor spinner;
    private DcMotor inOut;
    private Servo bucketRight;
    private Servo bucketLeft;
    private Servo grabberRight;
    private Servo grabberLeft;
    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        inOut = hardwareMap.dcMotor.get("inOut");
        lifter = hardwareMap.dcMotor.get("lifter");
        spinner = hardwareMap.dcMotor.get("spinner");
        bucketRight = hardwareMap.servo.get("bucketR");
        bucketLeft = hardwareMap.servo.get("bucketL");
        grabberRight = hardwareMap.servo.get("grabberR");
        grabberLeft = hardwareMap.servo.get("grabberL");
        inOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRightA.setDirection(DcMotor.Direction.REVERSE);      //think about logic of motors and how you need to reverse two of them
        //motorRightB.setDirection(DcMotor.Direction.REVERSE);
    }
    int speed = 1;
    @Override
    public void loop() {
        // goes into loop after the setup is done  in above void
        // sets up a loop for driving the robot
        /*
            TODO:
            * Take a look at documentation to see
            what can be changed
            * Change control layout -
                Right - rotate
                Left - strafe
                Dpad - Maybe remove
        */
        boolean dpadUp = gamepad1.dpad_up; //Bring arm and stuff out from dump
        boolean dpadDown = gamepad1.dpad_down; //Bring arm and stuff in for dump
        if(!gamepad1.atRest()){ //Make sure that a button is pressed before looping for performance reasons
            loopController1();
        }
        if(!gamepad2.atRest()){
            loopController2();
            if(dpadUp && isReturning){
                isReturning = false;
            }else if(dpadDown && isDumping){
                isDumping = false;
            }
        }
        if(dpadUp || isReturning){
            isReturning = true;
            if(!bucketReturning){ //Make sure not to continuously ask buckets to move
                bucketRight.setPosition(0.22); //return from dump
                bucketLeft.setPosition(0.77);
                bucketReturning = true;
            }else if(bucketRight.getPosition() < 0.29){ //Wait until bucket is close to correct position
                lifter.setTargetPosition(0);
                grabberRight.setPosition(1);
                grabberLeft.setPosition(0);
                inOut.setTargetPosition(1);
                spinner.setPower(1);
                bucketReturning = false;
                isReturning = false;
            }
        }else if(dpadDown || isDumping){
            isDumping = true;
            spinner.setPower(0); //Turn off and pull in spinner, dump, lift, dump
            if(!inOutDumping){ //Don't keep resetting target position while it is already set
                inOut.setTargetPosition(0);
                inOutDumping = true;
            }else if(!inOut.isBusy()){ //Wait for position to be reached
                if(!grabbersDumping){
                    grabberRight.setPosition(0.14);
                    grabberLeft.setPosition(0.86);
                    grabbersDumping = true;
                }else if(grabberRight.getPosition() < 0.18){
                    if(!lifterDumping){
                        lifter.setTargetPosition(1);
                        lifterDumping = true;
                    }else if(!lifter.isBusy()){
                        bucketRight.setPosition(1);
                        bucketLeft.setPosition(0);
                        isDumping = false;
                        inOutDumping = false;
                        grabbersDumping = false;
                        lifterDumping = false;
                    }
                }
            }
        }
    }
    //Revised based on https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    private void loopController1(){
        // input joystick values into variables that we can use to control the motors
        double rightX = gamepad1.right_stick_x; //Strafe
        double rightY = gamepad1.right_stick_y; //Speed
        double leftX = gamepad1.left_stick_x; //Yaw (Turn)
        boolean rightBumper = gamepad1.right_bumper; //Make spinner arm go out
        boolean leftBumper = gamepad1.left_bumper; //Make spinner arm go in
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);// sets a value check to make sure we don't go over the desired speed (related to joysticks)
        leftX = Range.clip(leftX, -1, 1);

        // Speed adjustment
        if (gamepad1.dpad_up) speed = 4;
        else if (gamepad1.dpad_right) speed = 3;
        else if (gamepad1.dpad_down) speed = 2;
        else if (gamepad1.dpad_left) speed = 1;

        if (Math.abs(rightX) > .1 || Math.abs(leftX) > .1 || Math.abs(rightY) > .1) { //Remember that left motor directions are reversed
            //Speed always positive, strafing wheels must be opposed, speed has to be opposite on left and right
            motorRightA.setPower((rightY + rightX + leftX) / 4 * speed);
            motorRightB.setPower((rightY - rightX + leftX) / 4 * speed);
            motorLeftA.setPower((-rightY + rightX + leftX) / 4 * speed); //Opposite (negative) of right because the motors are opposite
            motorLeftB.setPower((-rightY - rightX + leftX) / 4 * speed);
            //Strafe is not negative version because to strafe, on side goes out (<- ->), one goes in (-> <-)
            //Forward       Turn        Strafe
            /*  -> ->       -> ->       -> <-
            *   -> ->       <- <-       <- ->
            *  Think about force vectors created by 45° rotation on mecanum wheels to understand strafing
            * */
        } else { //will not move if joysticks are not moving
            motorRightA.setPower(0);
            motorRightB.setPower(0);
            motorLeftA.setPower(0);
            motorLeftB.setPower(0);
        }
        if(rightBumper){
            inOut.setTargetPosition(1);
        }else if(leftBumper){
            inOut.setTargetPosition(0);
        }
        telemetry.addData("Joy1", "Joystick 1:  " + String.format("%.2s", gamepad1.left_stick_y)); // feedback given to the driver phone from the robot phone
        telemetry.addData("Joy2", "Joystick 2:  " + String.format("%.2s", gamepad1.right_stick_y));
    }
    private void loopController2(){
        double rightY = gamepad2.right_stick_y; //Controls lifter arm, continuous
        double leftY = gamepad2.left_stick_y; //Controls grabber/spinner arm, not continuous
        float rightTrigger = gamepad2.right_trigger; //Turns on spinner
        float leftTrigger = gamepad2.left_trigger; //Turns on spinner in reverse
        //This is boolean because it is checking whether button is pressed
        boolean rightBumper = gamepad2.right_bumper; //Move bucket down
        boolean leftBumper = gamepad2.left_bumper; //Move bucket up
        rightY = Range.clip(rightY, -1, 1);
        leftY = Range.clip(leftY, -1, 1);
        if(Math.abs(leftY) > 0.1){ //Threshold to prevent response to controller bumps
            double position = Range.clip((leftY + 1)/2, 0.14, 1); //So that position is not negative and between 0 and 1
            double position2 = Range.clip((-leftY + 1)/2, 0, 0.86); //Opposing servos must have opposite directions
            grabberRight.setPosition(position);
            grabberLeft.setPosition(position2);
            telemetry.addData("Right servo position:", position); //For debugging, can be removed later
            telemetry.addData("Left servo position", position);
        }
        if(Math.abs(rightY) > 0.1){
            if(rightY > 0){
                lifter.setTargetPosition(1);
            }else if(rightY < 0){
                lifter.setTargetPosition(-1);
            }
        }
        if(rightBumper){
            bucketRight.setPosition(1);
            bucketLeft.setPosition(0);
        }else if(leftBumper){
            bucketRight.setPosition(0.22); //Minimum position is 45 degrees, so this goes to 40
            bucketLeft.setPosition(0.77);
        }
        if(rightTrigger > 0.1){
            if(spinner.getPower() > 0){
                spinner.setPower(0);
            }else{
                spinner.setPower(1);
            }
        }else if(leftTrigger > 0.1) {
            if(spinner.getPower() > 0){
                spinner.setPower(0);
            }else{
                spinner.setPower(-1);
            }
        }
    }
}