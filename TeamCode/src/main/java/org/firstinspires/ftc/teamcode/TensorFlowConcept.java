package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.List;

public class TensorFlowConcept extends LinearOpMode {

    private Robot bot;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private final String STONE_LABEL = "Stone";
    private final String SKYSTONE_LABEL = "SkyStone";

    private final double Y_TURN = 0;
    private final double Y_BLOCKS = 0;

    private int step = 0;
    private double power = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{32, 32, 32, 32},
                new double[]{560, 560, 560, 560},
                1,
                true
        );

        initVuforia();

        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            initTfod();
        }else{
            telemetry.addData("Error:", "This device is not compatible with TFOD");
        }

        waitForStart();

        if(tfod != null){
            tfod.activate();

            getStones();
            moveFoundation();
            park();

            tfod.shutdown();
        }
    }

    private void park(){

    }

    private void moveFoundation(){
        // Move bumpers down

        bot.rotateWithIMU(0.2, 50);
    }

    private void getStones(){
        final double MAX_DISTANCE_RATIO = 0.9;

        // Go left until close enough to row of blocks
        double currentDistanceRatio = 0;
        while(currentDistanceRatio < MAX_DISTANCE_RATIO){
            bot.drive(0.2, Robot.Direction.LEFT);

            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if(recognitions != null){
                currentDistanceRatio = getClosestRecognitionDistance(recognitions);
            }
        }
        bot.stop();

        int blocksFound = 0;
        final int MAX_BLOCKS = 3;

        double dy = lookForStone(true, true);
        deliverStone(true, dy, true);
        blocksFound++;

        while(blocksFound <= MAX_BLOCKS){
            dy = lookForStone(blocksFound <= 2, false);
            deliverStone(false, dy, blocksFound != MAX_BLOCKS);
            blocksFound++;
        }
    }

    private double lookForStone(boolean skystone, boolean fromBottom){
        boolean detected = false;
        Robot.Direction direction = fromBottom ? Robot.Direction.BACKWARD : Robot.Direction.FORWARD;
        String label = skystone ? SKYSTONE_LABEL : STONE_LABEL;

        double y0 = bot.dcMotors.get("mRF").getCurrentPosition();
        while(!detected){
            bot.drive(power, direction);

            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            detected = recognitions != null && containsLabel(recognitions, label); //Make sure this isn't calling method on a null reference
        }
        double yf = bot.dcMotors.get("mRF").getCurrentPosition();
        bot.stop();

        Double[] info = bot.dcMotorInfo.get("mRF");
        double dy = (yf - y0)*info[0]/info[1]; // Y distance travelled while looking for a block

        return dy;
    }

    private void deliverStone(boolean stoneFromBottom, double dy, boolean goBack){
        if(stoneFromBottom){
            bot.drive(power, Y_TURN + Y_BLOCKS - dy, Robot.Direction.BACKWARD);
        }else{
            bot.drive(power, Y_TURN + dy, Robot.Direction.BACKWARD);
        }

        if(goBack) bot.drive(power, Y_TURN, Robot.Direction.FORWARD);
    }

    private boolean containsLabel(List<Recognition> recognitions, String label){
        for(Recognition recognition : recognitions){
            if(recognition.getLabel().equals(label)){
                return true;
            }
        }

        return false;
    }

    private double getClosestRecognitionDistance(List<Recognition> recognitions){
        double min = -1;

        for(Recognition recognition : recognitions){
            double distRatio = recognition.getHeight()/recognition.getImageHeight();
            if(min < 0 || distRatio < min){
                min = distRatio;
            }
        }

        return min;
    }

    private void initVuforia(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    private void initTfod(){
        int tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewID);

        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_LABEL, SKYSTONE_LABEL);
    }

}
