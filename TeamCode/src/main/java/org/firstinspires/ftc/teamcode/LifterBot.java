package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Sam on 11/2/2017. Copied and pasted by Willem. Viewed by Milo.
 */
@TeleOp
//Using inheritance to extend other methods
public class LifterBot extends OpMode{
    DcMotor motorRightA;                    // creates motors in code
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    Servo liftRight;
    Servo liftLeft;
    Servo grabberRight;
    Servo grabberLeft;
    public void init() { // initiates and maps motors/servos/sensors
        motorRightA = hardwareMap.dcMotor.get("mRF");               // Finds the motor and the library to use it
        motorRightB = hardwareMap.dcMotor.get("mRB");               // !! Must be prenamed in phone app to green letters (mRF, mRB, etc.) !!
        motorLeftA = hardwareMap.dcMotor.get("mLF");
        motorLeftB = hardwareMap.dcMotor.get("mLB");
        liftRight = hardwareMap.servo.get("liftR");
        liftLeft = hardwareMap.servo.get("liftL");
        grabberRight = hardwareMap.servo.get("grabberR");
        grabberLeft = hardwareMap.servo.get("grabberL");
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
        loopController1();
        loopController2();
    }
    private void loopController1(){
        // input joystick values into variables that we can use to control the motors
        double rightX = -gamepad1.right_stick_x;
        double rightY = -gamepad1.left_stick_y; //Strafe
        double leftX = -gamepad1.left_stick_x; //Yaw
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);// sets a value check to make sure we don't go over the desired speed (related to joysticks)
        leftX = Range.clip(leftX, -1, 1);
        if (gamepad1.dpad_up) speed = 4;
        else if (gamepad1.dpad_right) speed = 3;
        else if (gamepad1.dpad_down) speed = 2;
        else if (gamepad1.dpad_left) speed = 1;
        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_y) > .1) {
            motorRightA.setPower((-rightY - rightX - leftX) / 4 * speed);
            motorRightB.setPower((-rightY + rightX + leftX) / 4 * speed);
            motorLeftA.setPower((rightY + rightX - leftX) / 4 * speed);
            motorLeftB.setPower((rightY - rightX + leftX) / 4 * speed);
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
        double rightY = -gamepad2.right_stick_y;
        double leftY = -gamepad2.left_stick_y;
        
    }
}