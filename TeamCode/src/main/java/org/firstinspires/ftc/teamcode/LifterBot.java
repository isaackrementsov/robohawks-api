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
    DcMotor motorRightA;                    // creates motors in code
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor lifter;
    DcMotor spinner;
    DcMotor inOut;
    Servo bucketRight;
    Servo bucketLeft;
    Servo grabberRight;
    Servo grabberLeft;
    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        inOut = hardwareMap.dcMotor.get("inOut");
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
    public void loop() {                                        // goes into loop after the setup is done  in above void
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
        if(!gamepad1.atRest()){ //Make sure that a button is pressed before looping for performance reasons
            loopController1();
        }
        if(!gamepad2.atRest()){
            loopController2();
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

        // Speed adjustment
        if (gamepad1.dpad_up) speed = 4;
        else if (gamepad1.dpad_right) speed = 3;
        else if (gamepad1.dpad_down) speed = 2;
        else if (gamepad1.dpad_left) speed = 1;

        if (Math.abs(rightX) > .1 || Math.abs(leftX) > .1 || Math.abs(rightY) > .1) {
            motorRightA.setPower((rightY + rightX + leftX) / 4 * speed);
            motorRightB.setPower((rightY - rightX + leftX) / 4 * speed);
            motorLeftA.setPower((-rightY + rightX + leftX) / 4 * speed);
            motorLeftB.setPower((-rightY - rightX + leftX) / 4 * speed);
        } else { //will not move if joysticks are not moving
            motorRightA.setPower(0);
            motorRightB.setPower(0);
            motorLeftA.setPower(0);
            motorLeftB.setPower(0);
        }
        telemetry.addData("Joy1", "Joystick 1:  " + String.format("%.2s", gamepad1.left_stick_y)); // feedback given to the driver phone from the robot phone
        telemetry.addData("Joy2", "Joystick 2:  " + String.format("%.2s", gamepad1.right_stick_y));
    }
    private void loopController2(){
        double rightY = -gamepad2.right_stick_y; //Controls lifter arm, continuous
        double leftY = -gamepad2.left_stick_y; //Controls grabber/spinner arm, not continuous
        //This is boolean because it is checking whether button is pressed
        boolean a = gamepad2.a; //Turns on spinner
        boolean b = gamepad2.b; //Turns on spinner in reverse
        boolean rightBumper = gamepad2.right_bumper; //button to reverse dump
        boolean leftBumper = gamepad2.left_bumper; //button to dump
        rightY = Range.clip(rightY, -1, 1);
        leftY = Range.clip(leftY, -1, 1);
        if(Math.abs(leftY) > .1){ //Threshold to prevent response to controller bumps
            double position = Range.clip((leftY + 1)/2, 0.14, 1); //So that position is not negative and between 0 and 1
            double position2 = Range.clip((-leftY + 1)/2, 0.14, 1);
            grabberRight.setPosition(position);
            grabberLeft.setPosition(position2);
            telemetry.addData("Right servo position:", position);
            telemetry.addData("Left servo position", position);
        }
        if(Math.abs(rightY) > .1){
            if(rightY > 0){
                lifter.setTargetPosition(1);
            }else if(rightY < 0){
                lifter.setTargetPosition(-1);
            }
        }
        if(rightBumper){
            bucketRight.setPosition(1);
            bucketLeft.setPosition(0);
        }
        if(leftBumper){
            bucketRight.setPosition(0);
            bucketLeft.setPosition(1);
        }
        /*if(a){
            if(spinner.getPower() > 0){
                spinner.setPower(0);
            }else{
                spinner.setPower(1);
            }
        }
        if(b){
            if(spinner.getPower() > 0){
                spinner.setPower(0);
            }else{
                spinner.setPower(-1);
            }
        }*/
    }
}