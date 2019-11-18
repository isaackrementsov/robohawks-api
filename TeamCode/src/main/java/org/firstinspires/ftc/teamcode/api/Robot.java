package org.firstinspires.ftc.teamcode.api;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private String[] drivetrain = new String[4];

    public HashMap<String, DcMotor> dcMotors = new HashMap<>();
    private HashMap<String, Double[]> dcMotorInfo = new HashMap<>();
    public HashMap<String, Servo> servos = new HashMap<>();
    private HashMap<String, Double[]> servoLimits = new HashMap<>();
    public HashMap<String, TouchSensor[]> limitSwitches = new HashMap<>();
    public HashMap<String, DistanceSensor> distanceSensors = new HashMap<>();
    public HashMap<String, ColorSensor> colorSensors = new HashMap<>();

    public void addDrivetrain(String[] motors, double[] circumferences, double[] encoderTicks, double[] radii, boolean reverseLeft){
        drivetrain = motors;
        addDcMotors(motors, circumferences, encoderTicks, radii);

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
        addDrivetrain(motors, new double[]{1,1,1,1}, new double[]{0,0,0,0}, new double[]{0,0,0,0}, reverseLeft);
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

    public void rotate(double power, double angle) {
        double[] powers = new double[4];

        for (int i = 0; i < drivetrain.length; i++) {
            double motorPower = 0.0;
            String motor = drivetrain[i];

            switch (i) {
                case 0:
                    motorPower = power;
                    break;
                case 1:
                    motorPower = -power;
                    break;
                case 2:
                    motorPower = power;
                    break;
                case 3:
                    motorPower = -power;
                    break;
            }

            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];
            double radius = info[2];

            int target = (int) (angle * Math.PI/180 * encoderTicks * radius / circumference);

            telemetry.addData("Target pos: ", /*(motorPower < 0 ? -1 : 1) * */ target + dcMotors.get(motor).getCurrentPosition());
            telemetry.update();

            dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) *  target + dcMotors.get(motor).getCurrentPosition());
            dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);

            powers[i] = motorPower;
        }

        for (int i = 0; i < drivetrain.length; i++) {
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        while(dcMotors.get(drivetrain[0]).isBusy());
    }

    public void stop(){
        drive(0, 0, 0, 0);
    }

    public void addDcMotor(String motor, double circumference, double encoderTicks, double radius){
        dcMotors.put(motor, hardwareMap.dcMotor.get(motor));
        dcMotorInfo.put(motor,  new Double[]{circumference, encoderTicks, radius});

        dcMotors.get(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void addDcMotors(String motors[], double[] circumferences, double[] encoderTicks, double[] radii){
        for(int i = 0; i < motors.length; i++){
            addDcMotor(motors[i], circumferences[i], encoderTicks[i], radii[i]);
        }
    }

    public void moveDcMotor(String motor, double distanceCM, double motorPower){
        Double[] info = dcMotorInfo.get(motor);

        double circumference = info[0];
        double encoderTicks = info[1];
        int target = (int) (encoderTicks * distanceCM / circumference);

        dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());
        dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotors.get(motor).setPower(motorPower);

        while(dcMotors.get(motor).isBusy());
    }

    public void addLimitedMotor(String motor, String limitLower, String limitUpper, double circumference, double encoderTicks){
        addDcMotor(motor, circumference, encoderTicks, 0);

        TouchSensor sensorLower = hardwareMap.touchSensor.get(limitLower);
        TouchSensor sensorUpper = hardwareMap.touchSensor.get(limitUpper);
        limitSwitches.put(motor, new TouchSensor[]{sensorLower, sensorUpper});
    }

    public void moveLimitedMotor(String motor, double distanceCM, double motorPower){
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor sensor = limitSwitches.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);

        Double[] info = dcMotorInfo.get(motor);
        double circumference = info[0];
        double encoderTicks = info[1];
        int target = (int) (encoderTicks * distanceCM / circumference);

        double l = 7.62;
        double m = 4/3 * circumference/(l*encoderTicks);
        double dn = 0.0001;
        double slow = l*motorPower*encoderTicks/circumference;

        motorToMove.setTargetPosition(sign * target + dcMotors.get(motor).getCurrentPosition());

        while(Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition()) > 10 && !sensor.isPressed()){
            double diff = Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition());
            double dp = (diff <= slow && motorPower > 0) ? m*dn : 0;
            motorPower -= dp;

            motorToMove.setPower(motorPower);
        }

        motorToMove.setPower(0);
    }

    public void resetLimitedMotor(String motor){
        TouchSensor endSensor = limitSwitches.get(motor)[1];
        DcMotor motorToReset = dcMotors.get(motor);

        while(!endSensor.isPressed()){
            motorToReset.setPower(-0.2);
        }

        motorToReset.setPower(0);
        motorToReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void addServo(String motor){
        addServo(motor, 180, 180, -180);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle){
        servos.put(motor, hardwareMap.servo.get(motor));
        servoLimits.put(motor, new Double[]{rotationAngle, minAngle, maxAngle});
    }

    public void resetServo(String motor, int waitTimeMillis){
        servos.get(motor).setPosition(servoLimits.get(motor)[1]);
        waitMillis(waitTimeMillis);
    }

    public void rotateServo(String motor, double angle, int waitTimeMillis){
        Double[] angleLimits = servoLimits.get(motor);

        double rotationAngle = angleLimits[0];
        double minAngle = angleLimits[1];
        double maxAngle = angleLimits[2];

        if(angle >= minAngle && angle <= maxAngle){
            servos.get(motor).setPosition(angle/rotationAngle);
            waitMillis(waitTimeMillis);
        }
    }

    public void holdServo(String motor, double angle, int waitTimeMillis){
        long start = System.currentTimeMillis();

        while(System.currentTimeMillis() - start < waitTimeMillis){
            rotateServo(motor, angle, 0);
        }
    }

    private void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) { }
    }

    public void addColorSensor(String sensor){
        colorSensors.put(sensor, hardwareMap.colorSensor.get(sensor));
    }

    public int[] getColorRGB(String sensor){
        ColorSensor sensorToUse = colorSensors.get(sensor);

        return new int[]{sensorToUse.red(), sensorToUse.green(), sensorToUse.blue()};
    }

    public void addDistanceSensor(String sensor, boolean optical){
        if(optical){
            distanceSensors.put(sensor, (DistanceSensor) hardwareMap.opticalDistanceSensor.get(sensor));
        }else{
            distanceSensors.put(sensor, (DistanceSensor) hardwareMap.colorSensor.get(sensor));
        }
    }

    public double getDistanceCM(String sensor){
        DistanceSensor sensorToUse = distanceSensors.get(sensor);

        return sensorToUse.getDistance(DistanceUnit.CM);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public static enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }
}
