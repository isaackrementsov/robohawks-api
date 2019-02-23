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
public class LifterBot_no_auto extends OpMode {
    private int adjustmentForGrabbers = 1; //Change to 2 if using continuous (or 360) servos
    private static final int LIFTER_TICK_COUNTS = 1120; //Tick counts for encoded motors
    private static final int INOUT_TICK_COUNTS = 288;
    private DcMotor motorRightA;                    // creates motors in code
    private DcMotor motorRightB;
    private DcMotor motorLeftA;
    private DcMotor motorLeftB;
    private DcMotor lifter; //1 rev = 2 inches, 8.5 inch range, will start at top
    private DcMotor spinner;
    private DcMotor inOut;
    private Servo bucketRight;
    private Servo bucketLeft;
    private Servo grabberRight;
    private Servo grabberLeft;
    private Servo locker;
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
        locker = hardwareMap.servo.get("locker");
        locker.setPosition(0.36); //Unlock lifter
      //  inOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  inOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorRightA.setDirection(DcMotor.Direction.REVERSE);      //think about logic of motors and how you need to reverse two of them
        //motorRightB.setDirection(DcMotor.Direction.REVERSE);
    }
    int speed = 1;
    @Override
    public void loop() {
        // goes into loop after the setup is done  in above void
        // sets up a loop for driving the robot
        if(!gamepad1.atRest()){//Make sure that a button is pressed before looping for performance reasons
            loopController1(); //Handles gamepad1 controls
        }
        if(!gamepad2.atRest()){
            loopController2(); //Handles gamepad2 controls
        }
        loopBooleanButtons(); //Handles controls not picked up in atRest(), which are boolean buttons (not triggers or joysticks)
    }
    //Revised based on https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    private void loopController1(){
        // input joystick values into variables that we can use to control the motors
        double rightX = gamepad1.right_stick_x; //Strafe
        double rightY = gamepad1.right_stick_y; //Speed
        double leftX = gamepad1.left_stick_x; //Yaw (Turn)
        float rightTrigger = gamepad1.right_trigger;
        float leftTrigger = gamepad1.left_trigger;
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
                -> ->       <- <-       <- ->
               Think about force vectors created by 45° rotation on mecanum wheels to understand strafing
            */
        } else { //will not move if joysticks are not moving
            motorRightA.setPower(0);
            motorRightB.setPower(0);
            motorLeftA.setPower(0);
            motorLeftB.setPower(0);
        }
        if(rightTrigger > 0.1){
            lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lifter.setPower(-0.14);
        }else if(leftTrigger > 0.1){
            lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lifter.setPower(leftTrigger/1.5);
        }else{
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setPower(0);
        }
        telemetry.addData("Joy1", "Joystick 1:  " + String.format("%.2s", gamepad1.left_stick_y)); // feedback given to the driver phone from the robot phone
        telemetry.addData("Joy2", "Joystick 2:  " + String.format("%.2s", gamepad1.right_stick_y));
    }
    private void loopController2(){
        double rightY = gamepad2.right_stick_y; //Controls lifter arm, continuous
        double leftY = gamepad2.left_stick_y; //Controls grabber/spinner arm, not continuous
        float rightTrigger = gamepad2.right_trigger; //Turns on spinner
        float leftTrigger = gamepad2.left_trigger; //Turns on spinner in reverse
        rightY = Range.clip(rightY, -1, 1);
        leftY = Range.clip(leftY, -1, 1);
        if(Math.abs(leftY) > 0.1){ //Threshold to prevent response to controller bumps
            double position2 = Range.clip((leftY + 1)/2, 0.14, 1); //So that position is not negative and between 0 and 1
            double position = Range.clip((-leftY + 1)/2, 0, 0.86); //Opposing servos must have opposite directions
            grabberRight.setPosition((float) position/adjustmentForGrabbers);
            grabberLeft.setPosition((float) position2/adjustmentForGrabbers);
        }
        if(rightY > 0.1) {
            inOut.setPower(rightY);
        }else if(rightY < 0.2){
            inOut.setPower(rightY);
        }else{
            inOut.setPower(0);
        }
        if(rightTrigger > 0.2){
            spinner.setPower(-1/1.5);
        }else if(leftTrigger > 0.1){
            spinner.setPower(1/1.5);
        }else{
            spinner.setPower(0);
        }
    }
    private void loopBooleanButtons(){
        //This is boolean because it is checking whether button is pressed
    //    boolean rightBumper = gamepad1.right_bumper; //Make spinner arm go out
    //    boolean leftBumper = gamepad1.left_bumper; //Make spinner arm go in
        boolean rightBumper2 = gamepad2.right_bumper; //Move bucket down
        boolean leftBumper2 = gamepad2.left_bumper; //Move bucket up
    //    boolean y = gamepad2.y; //Dedicated testing button
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean y = gamepad2.y;

        if(rightBumper2){
            bucketRight.setPosition(1);
            bucketLeft.setPosition(0);
        }else if(leftBumper2){
            bucketRight.setPosition(0.22); //Minimum position is 45 degrees, so this goes to 40
            bucketLeft.setPosition(0.77);
        }

        if(a){
            locker.setPosition(0);
        }
        if(b){
            locker.setPosition(0.36);
        }

    /*    if(rightBumper){
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setTargetPosition((int) Math.floor(-LIFTER_TICK_COUNTS*4.25));
        }else if(leftBumper) {
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setTargetPosition(0);
        }
        if((rightBumper || leftBumper) && inOut.isBusy()){
            lifter.setPower(0);
        }

    */

    }
}