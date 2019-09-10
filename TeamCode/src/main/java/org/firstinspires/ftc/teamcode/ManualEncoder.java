package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ManualEncoder extends OpMode {

    DcMotor motor;

    public void init(){
        motor = hardwareMap.dcMotor.get("mLF");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Initializing... ", "");
    }

    public void loop(){
        telemetry.addData("Current encoder position: ", motor.getCurrentPosition());
    }

}

