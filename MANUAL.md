# Programming Without Team API
In the case that the Team API is broken or you need 
to add features, this is your guide to using the lower-level
FTC API

## Encoders
Encoders are one of the most confusing and poorly 
documented parts of the FTC API. The following outlines
how to make encoders work.

### TeleOp
It's not intuitive, but you have to set the motor's runmode
to `DcMotor.RunMode.RUN_WITHOUT_ENCODER`. If you want to reset
the wheel position seen by the encoder, `DcMotor.RunMode.STOP_AND_RESET_ENCODER`.

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderOpMode extends OpMode {
    
    DcMotor motor;
    
    int TICK_COUNT = 1680; //Tick count for motor

    public void init(){
        motor = hardwareMap.dcMotor.get("mLF");
        
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void loop() {
        if(gamepad1.dpad_down) {
            runToPosition(1);
        }

        telemetry.addData("Current pos", motor.getCurrentPosition());
    }
    
    private void runToPosition(int revolutions){
        motor.setTargetPosition(TICK_COUNT*revolutions + motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
    
}
```

### Autonomous
Autonomous code is a bit simpler, but you should add a `while`
loop before executing any other code (if you don't want functions to execute while the bot is moving)

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonOpMode extends LinearOpMode {
    public void runOpMode(){
        DcMotor motor = hardwareMap.dcMotor.get("mLF");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        
        int TICKS = 1680; //Motor tick count
        motor.setTargetPosition(TICKS*2);
        
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Tells motor to stop at specified position
        motor.setPower(0.2); //How fast, on average, to go to the position
        while(motor.isBusy());
        
        //DO stuff after motor is finished
    }
}
```
