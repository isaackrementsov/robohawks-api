package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Robot {

    private HardwareMap hardwareMap;
    public String[] drivetrain = new String[4];
    public HashMap<String, DcMotor> dcMotors = new HashMap<>();
    public HashMap<String, Double[]> dcMotorInfo = new HashMap<>();
    public HashMap<String, Servo> servos = new HashMap<>();
    public HashMap<String, Double[]> servoLimits = new HashMap<>();

    public void addDrivetrain(String[] motors, double[] circumferences, double[] encoderTicks, boolean reverseLeft){
        drivetrain = motors;
        addDcMotors(motors, circumferences, encoderTicks);

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            DcMotor dcMotor = dcMotors.get(motor);

            if(i % 2 == 0 && reverseLeft){
                dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void addDrivetrain(String[] motors, boolean reverseLeft){
        addDrivetrain(motors, new double[]{1,1,1,1}, new double[]{0,0,0,0}, reverseLeft);
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
                    if(i == 0 || i == 3){
                        motorPower = -power;
                    }
                    break;
                case LEFT:
                    if (i == 1 || i == 2){
                        motorPower = -power;
                    }
                    break;
            }

            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];
            int target = (int) (encoderTicks * distanceCM / circumference);

            dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());
            dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                    break;
                case LEFT:
                    if(i < 2){
                        motorPower = -power;
                    }
            }

            dcMotors.get(motor).setPower(motorPower);
        }
    }

    public void drive(double speed, double yaw, double strafe, double power){
        for(int i = 0; i < drivetrain.length; i++){
            double motorPower = 0;
            String motor = drivetrain[i];

            switch(i){
                case 0:
                    motorPower = power + strafe + yaw;
                    break;
                case 1:
                    motorPower = power - strafe - yaw;
                    break;
                case 2:
                    motorPower = power - strafe + yaw;
                    break;
                case 3:
                    motorPower = power + strafe - yaw;
            }

            motorPower *= speed;

            dcMotors.get(motor).setPower(motorPower);
        }
    }

    public void stop(){
        drive(0, 0, 0, 0);
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
        addServo(motor, 180, 180, -180);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle){
        servos.put(motor, hardwareMap.servo.get(motor));
        servoLimits.put(motor, new Double[]{rotationAngle, minAngle, maxAngle});
    }

    public void resetServo(String motor){
        servos.get(motor).setPosition(0);
    }

    public void rotateServo(String motor, double angle){
        Double[] angleLimits = servoLimits.get(motor);

        double rotationAngle = angleLimits[0];
        double minAngle = angleLimits[1];
        double maxAngle = angleLimits[2];

        double currentAngle = servos.get(motor).getPosition()*rotationAngle;
        double targetAngle = currentAngle + angle;

        if(targetAngle >= minAngle && targetAngle <= maxAngle){
            servos.get(motor).setPosition(targetAngle/rotationAngle);
        }
    }

    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public static enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }
}
