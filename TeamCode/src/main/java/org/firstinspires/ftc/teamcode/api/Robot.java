package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public String[] drivetrain = new String[4];
    public HashMap<String, DcMotor> dcMotors = new HashMap<>();
    public HashMap<String, Double[]> dcMotorInfo = new HashMap<>();
    public HashMap<String, Servo> servos = new HashMap<>();
    public HashMap<String, Double[]> servoLimits = new HashMap<>();

    public void addDrivetrain(String[] motors, double[] circumferences, double[] encoderTicks){
        drivetrain = motors;
        addDcMotors(motors, circumferences, encoderTicks);

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            DcMotor dcMotor = dcMotors.get(motor);

            if(i % 2 == 0){
                dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive(double power, double distanceCM, Direction direction){
        double[] powers = new double[4];

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            double motorPower = power;

            switch(direction){
                case BACKWARD:
                    motorPower = -power;
                    break;
                case RIGHT:
                    if(i > 1){
                        motorPower = -power;
                    }
                case LEFT:
                    if(i < 2){
                        motorPower = -power;
                    }
            }

            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];

            dcMotors.get(motor).setTargetPosition((int) (encoderTicks * distanceCM / circumference) + dcMotors.get(motor).getCurrentPosition());
            dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dcMotors.get(motor).setPower(motorPower);

            powers[i] = motorPower;
        }

        for(int i = 0; i < drivetrain.length; i++){
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        while(dcMotors.get(drivetrain[0]).isBusy());
    }

    public void drive(double power, Direction direction){
        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            double motorPower = power;

            switch(direction){
                case BACKWARD:
                    motorPower = -power;
                    break;
                case RIGHT:
                    if(i > 1){
                        motorPower = -power;
                    }
                case LEFT:
                    if(i < 2){
                        motorPower = -power;
                    }
            }

            dcMotors.get(motor).setPower(motorPower);
        }
    }

    public void addDcMotor(String motor, double circumference, double encoderTicks){
        dcMotors.put(motor, hardwareMap.dcMotor.get(motor));
        dcMotorInfo.put(motor,  new Double[]{circumference, encoderTicks});
    }

    public void addDcMotors(String motors[], double[] circumferences, double[] encoderTicks){
        for(int i = 0; i < motors.length; i++){
            addDcMotor(motors[i], circumferences[i], encoderTicks[i]);
        }
    }

    public void addServo(String motor){
        addServo(motor, 180, 0);
    }

    public void addServo(String motor, double maxAngle, double minAngle){
        servos.put(motor, hardwareMap.servo.get(motor));
        servoLimits.put(motor, new Double[]{minAngle, maxAngle});
    }

    public void rotateServo(String motor, double angle){
        Double[] angleLimits = servoLimits.get(motor);

        if(angle > angleLimits[0] && angle < angleLimits[1]){
            servos.get(motor).setPosition(angle/360);
        }
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public static enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }
}
