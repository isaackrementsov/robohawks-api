package org.firstinspires.ftc.teamcode.api;

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
    private HashMap<String, Double> servoPositions = new HashMap<>();
    private HashMap<String, Double[]> servoLimits = new HashMap<>();

    public HashMap<String, TouchSensor[]> limitSwitches = new HashMap<>();
    public HashMap<String, TouchSensor[][]> limitSwitchArrays = new HashMap<>();

    public HashMap<String, DistanceSensor> distanceSensors = new HashMap<>();

    public HashMap<String, ColorSensor> colorSensors = new HashMap<>();
    private HashMap<String, Integer> colorSensorInfo = new HashMap<>();

    public double rotationCoefficient;

    public void addDrivetrain(String[] motors, double[] circumferences, double[] encoderTicks, double rotationalCoefficient, boolean reverseLeft){
        drivetrain = motors;
        rotationCoefficient = rotationalCoefficient;
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
        addDrivetrain(motors, new double[]{1,1,1,1}, new double[]{0,0,0,0}, 1, reverseLeft);
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

        for(int i = 0; i < drivetrain.length; i++){
            double motorPower = power;

            if(i == 1 || i == 3){
                motorPower = -power;
            }

            powers[i] = motorPower;

            String motor = drivetrain[i];
            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];

            int target = (int) (rotationCoefficient * angle * encoderTicks / circumference);
            dcMotors.get(motor).setTargetPosition(target);
        }

        DcMotor test = dcMotors.get(drivetrain[0]);
        while(Math.abs(test.getTargetPosition() - test.getCurrentPosition()) > 10){
            for(int i = 0; i < drivetrain.length; i++){
                dcMotors.get(drivetrain[i]).setPower(powers[i]);
            }
        }
    }

    public void stop(){
        drive(0, 0, 0, 0);
    }

    public void addDcMotor(String motor){ addDcMotor(motor, 0, 0); }

    public void addDcMotor(String motor, double circumference, double encoderTicks){
        dcMotors.put(motor, hardwareMap.dcMotor.get(motor));
        dcMotorInfo.put(motor,  new Double[]{circumference, encoderTicks});

        dcMotors.get(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void addDcMotors(String motors[], double[] circumferences, double[] encoderTicks){
        for(int i = 0; i < motors.length; i++){
            addDcMotor(motors[i], circumferences[i], encoderTicks[i]);
        }
    }

    public void moveDcMotor(String motor, double motorPower){
        DcMotor motorToMove = dcMotors.get(motor);

        motorToMove.setPower(motorPower);
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

    public void addLimitedMotor(String motor, String limitLower, String limitUpper){
        addLimitedMotor(motor, limitLower, limitUpper, 0, 0);
    }

    public void addLimitedMotor(String motor, String limitLower, String limitUpper, double circumference, double encoderTicks){
        addDcMotor(motor, circumference, encoderTicks);

        TouchSensor sensorLower = hardwareMap.touchSensor.get(limitLower);
        TouchSensor sensorUpper = hardwareMap.touchSensor.get(limitUpper);
        limitSwitches.put(motor, new TouchSensor[]{sensorLower, sensorUpper});
    }

    public void addLimitedMotor(String motor, String[] limitsLower, String[] limitsUpper){
        addLimitedMotor(motor, limitsLower, limitsUpper, 0, 0);
    }

    public void addLimitedMotor(String motor, String[] limitsLower, String[] limitsUpper, double circumference, double encoderTicks){
        addDcMotor(motor, circumference, encoderTicks);

        TouchSensor[] sensorsLower = new TouchSensor[limitsLower.length];

        for(int i = 0; i < limitsLower.length; i++){
            sensorsLower[i] = hardwareMap.touchSensor.get(limitsLower[i]);
        }

        TouchSensor[] sensorsUpper = new TouchSensor[limitsUpper.length];

        for(int i = 0; i < limitsUpper.length; i++){
            sensorsUpper[i] = hardwareMap.touchSensor.get(limitsUpper[i]);
        }

        limitSwitchArrays.put(motor, new TouchSensor[][]{sensorsLower, sensorsUpper});
    }

    public void moveLimitedMotor(String motor, double motorPower){
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor sensor = limitSwitches.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);

        if(sensor.isPressed()){
            motorToMove.setPower(0);
        }else{
            motorToMove.setPower(motorPower);
        }
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

    public void moveLimitedMotorArray(String motor, double motorPower, LimitBehavior behavior){
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor[] sensors = limitSwitchArrays.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);

        if(arePressed(sensors, behavior)){
            motorToMove.setPower(0);
        }else{
            motorToMove.setPower(motorPower);
        }
    }

    public void moveLimitedMotorArray(String motor, double distanceCM, double motorPower, LimitBehavior behavior) {
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor[] sensors = limitSwitchArrays.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);

        Double[] info = dcMotorInfo.get(motor);
        double circumference = info[0];
        double encoderTicks = info[1];
        int target = (int) (encoderTicks * distanceCM / circumference);

        double l = 7.62;
        double m = 4 / 3 * circumference / (l * encoderTicks);
        double dn = 0.0001;
        double slow = l * motorPower * encoderTicks / circumference;

        motorToMove.setTargetPosition(sign * target + dcMotors.get(motor).getCurrentPosition());

        while (Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition()) > 10 && arePressed(sensors, behavior)) {
            double diff = Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition());
            double dp = (diff <= slow && motorPower > 0) ? m * dn : 0;
            motorPower -= dp;

            motorToMove.setPower(motorPower);
        }

        motorToMove.setPower(0);
    }

    private boolean arePressed(TouchSensor[] sensors, LimitBehavior behavior){
        boolean pressed = behavior == LimitBehavior.AND;

        for(TouchSensor sensor : sensors){
            if(behavior == LimitBehavior.AND) pressed = sensor.isPressed() && pressed;
            if(behavior == LimitBehavior.OR) pressed = sensor.isPressed() || pressed;
        }

        telemetry.addData("pressed", pressed);
        telemetry.update();

         return pressed;
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

    public void resetLimitedMotorArray(String motor, LimitBehavior behavior) {
        TouchSensor[] endSensors = limitSwitchArrays.get(motor)[1];
        DcMotor motorToReset = dcMotors.get(motor);

        while(arePressed(endSensors, behavior)){
            motorToReset.setPower(-0.2);
        }

        motorToReset.setPower(0);
        motorToReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void addServo(String motor){
        addServo(motor, 180, 180, 0);
    }

    public void addServo(String motor, double startAngle){
        servoPositions.put(motor, startAngle);

        addServo(motor);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle, double startAngle){
        servoPositions.put(motor, startAngle);

        addServo(motor, rotationAngle, maxAngle, minAngle);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle){
        servos.put(motor, hardwareMap.servo.get(motor));
        servoLimits.put(motor, new Double[]{rotationAngle, minAngle, maxAngle});
    }

    public void resetServo(String motor, int waitTimeMillis){
        Double startAngle = servoPositions.get(motor);

        servos.get(motor).setPosition((startAngle == null) ? servoLimits.get(motor)[1] : startAngle);
        waitMillis(waitTimeMillis);
    }

    public void incrementServo(String motor, double increment, int waitTimeMillis){
        Double startAngle = servoPositions.get(motor);
        double target = startAngle + increment;

        Double[] limits = servoLimits.get(motor);

        if(startAngle != null && target <= limits[1] && target >= limits[0]) {
            servos.get(motor).setPosition(target);
            servoPositions.put(motor, target);
        }
    }

    public void rotateServo(String motor, double angle, int waitTimeMillis){
        Double[] angleLimits = servoLimits.get(motor);

        double rotationAngle = angleLimits[0];
        double minAngle = angleLimits[1];
        double maxAngle = angleLimits[2];

        if(angle >= minAngle && angle <= maxAngle){
            servos.get(motor).setPosition(angle/rotationAngle);

            Double currentAngle = servoPositions.get(motor);
            if(currentAngle != null) servoPositions.put(motor, angle);

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

    public void addColorSensor(String sensor, int scaleFactor){
        colorSensors.put(sensor, hardwareMap.get(ColorSensor.class, sensor));
        colorSensorInfo.put(sensor, scaleFactor);
    }

    public int[] getColorRGBA(String sensor){
        ColorSensor sensorToUse = colorSensors.get(sensor);
        int scaleFactor = colorSensorInfo.get(sensor);

        return new int[]{
            scaleFactor*sensorToUse.red(),
            scaleFactor*sensorToUse.green(),
            scaleFactor*sensorToUse.blue(),
            scaleFactor*sensorToUse.alpha()
        };
    }

    public void addDistanceSensor(String sensor, boolean optical){
        if(optical){
            distanceSensors.put(sensor, (DistanceSensor) hardwareMap.opticalDistanceSensor.get(sensor));
        }else{
            distanceSensors.put(sensor, hardwareMap.get(DistanceSensor.class, sensor));
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

    public static enum LimitBehavior {
        AND, OR
    }
}
