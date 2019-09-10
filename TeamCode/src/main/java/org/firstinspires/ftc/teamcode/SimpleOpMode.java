package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SimpleOpMode extends OpMode {
    DcMotor motor;
    int TICK_COUNT = 1680;
    int target;
    boolean revStart = false;
    public void init(){
        motor = hardwareMap.dcMotor.get("mLF");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {
        boolean control = gamepad1.dpad_up;
        boolean control2 = gamepad1.dpad_down;
        if(control){
            telemetry.addData("Rev start", revStart);
        }
        if(control && !revStart){
            revStart = true;
            target = TICK_COUNT + motor.getCurrentPosition();
            //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.2);
            telemetry.addData("Setting power: ", motor.getPowerFloat());
        }
        if(revStart && Math.abs(motor.getCurrentPosition() - target) < 20){
            motor.setPower(0);
            revStart = false;
        }
        if(control2) {
            runToPosition();
        }

        telemetry.addData("Current pos", motor.getCurrentPosition());
    }
    private void runToPosition(){
        motor.setTargetPosition(TICK_COUNT + motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
}
