package UltimateGoal_RobotTeam.HarwareConfig;


//import OfflineCode.OfflineHW.CameraSimulant;//NEEDED FOR OFFLINE

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class ImageRecog {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = " AUtTfjH/////AAAAGalBbST5Vkv8kqIoszZrsYOCBYcUVlFwYJ2rvrvVgm4ie/idQYx2x++SWi3UMEJnuP7Zww+cqOgyLepotRs9ppGxpCDcintIz0pIixMr+bievjJUDzdn0PyAJ6QUZK3xzoqDkTY1R99qvRORmTTqCx2/rGfYPlcOpdL5oWdhQsUatHyneF/eiWGBincPqBx3JfVwZnscF/B7J+u76YA4VTWLl8bl3iu26IYXMZE0zi7Pk+s9/pRQCCrLcxsvns4ouiSg3n61Z+jv8aS6y9ncwDdg0nm/XwDeiIrworkIPcPTW73LKpzX/63C1HprikXUJ+fm1eAkCfNy06n9SNTq20jxc7NXtGVUoE+WbNGmE4yb ";

    private int tfodMonitorViewId = 0;
    /* Public OpMode members. */
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    public TFObjectDetector.Parameters tfodParameters = null;
    private List<Recognition> ringRecognitions;

    public ImageRecog(BasicOpMode om, boolean tm)  {
        if(tm) {
            om.telemetry.addData("Image Recognition", " Initializing in Test Mode...");
            om.telemetry.update();

//            tfod = new CameraSimulant();//NEEDED FOR OFFLINE

            if (tfod != null) {
                tfod.activate();
                om.telemetry.addLine("Status ... Simulant TFOD activated");
                om.telemetry.update();

            }
            om.telemetry.addLine("\t\t... Initialization COMPLETE");

            om.telemetry.update();
        }
        else {

            /**
             * Initialize the Vuforia localization engine within the constructor.  "This was initVuforia"
             */
            /**
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            om.telemetry.addData("Image Recognition", " Initializing ...");
            om.telemetry.update();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;// FRONT screen side, BACK = back of phone

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.

            /**
             * Initialize the TensorFlow Object Detection engine = tfod.  This was "initTfod"
             */
            tfodMonitorViewId = om.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", om.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;// Set the confidence factor
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 1.78 or 16/9).

                // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                //tfod.setZoom(2.5, 1.78);
                om.telemetry.addLine("Status ... TFOD activated");
                om.telemetry.update();

            }
            om.telemetry.addLine("\t\t... Initialization COMPLETE");

            om.telemetry.update();
        }

    }
public void getTelemetry(BasicOpMode om){
    ringRecognitions = tfod.getRecognitions();
    for (int i = 0; i < ringRecognitions.size(); i++) {
        om.telemetry.addData(String.format("\tLabel (%d)", i), ringRecognitions.get(i).getLabel());
        om.telemetry.addData(String.format("\t\tleft,top (%d)", i), "%.03f , %.03f",
                ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
        om.telemetry.addData(String.format("\t\tright,bottom (%d)", i), "%.03f , %.03f",
                ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());
    }
    // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
    // Can add more lines as needed or max a BASIC and MAX TM option
}
    public void shutdown(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public String viewRings(BasicOpMode om, int loopsToExit){
        String imageType = "none";
        int loops = 0; // run through iterations to confirm a single stable image or no image
        List<Recognition> ringRecognitions = tfod.getRecognitions();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int prevSize = ringRecognitions.size();
        if (tfod != null) {
            while ((loops < loopsToExit) && om.opModeIsActive()){ // goal is ten loops with the same number of images, could use updated recognitions for 10 loops of no new info
                ringRecognitions = tfod.getRecognitions();
                updatedRecognitions = tfod.getUpdatedRecognitions();
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.  Will be null if we're seeing the same thing so
                // we can use this to count loops with no change - might be hard to get "no change" even with robot being still
                // getRecognitions() will return the current stored recognitions which is what we want
                // if we want to output the image info
                if (ringRecognitions == null) { //not seeing any images
                    om.telemetry.addData("# Object Detected", 0);
                    imageType = "none";
                    if (prevSize == 0){
                        loops++; // if we've been seeing 0 images then keep counting the loops
                    }
                    else {
                        loops = 0;// if this is the first time to see zero images then reset
                    }
                    prevSize = 0;// set the value for "previous" after comparison
                }
                else {
                    om.telemetry.addData("# Object Detected", ringRecognitions.size());

                    if(ringRecognitions.size() == 1) {
                        imageType = ringRecognitions.get(0).getLabel();// only update the value to the label if there's a single image
                    }
                    else if (ringRecognitions.size() == 0){
                        imageType = "None";
                    }
                    else {
                        imageType = "Multiple";
                    }
                    if (prevSize == ringRecognitions.size()){
                        loops++; // if we've been seeing the same number of images then keep counting the loops
                    }
                    else {
                        loops = 0;// if this is the first time to see this number images then reset
                    }
                    prevSize = ringRecognitions.size(); // set the value for "previous" after comparison
                    // step through the list of recognitions and display boundary info.
                    for (int i = 0; i < ringRecognitions.size(); i++) {
                        om.telemetry.addData(String.format("label (%d)", i), ringRecognitions.get(i).getLabel());
                        om.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
                        om.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());
                    }
                }
                om.telemetry.addData("Loop counter", loops);
                om.telemetry.update();// update the telemetry
            }
            if(ringRecognitions == null) { //not seeing any images
                om.telemetry.addData("# Object Detected", 0);
            }
            else {
                om.telemetry.addData("# Object Detected", ringRecognitions.size());
                for (int i = 0; i < ringRecognitions.size(); i++) {
                    om.telemetry.addData(String.format("label (%d)", i), ringRecognitions.get(i).getLabel());
                    om.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
                    om.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());

                }
            }
            om.telemetry.addData("Image Type Returned", imageType);
            om.telemetry.addData("Loop counter", loops);
        }

        return imageType;
    }
    public String viewRingsTimed(BasicOpMode om, double timeToExit){
        String imageType = "none";
        int loops = 0; // run through iterations to confirm a single stable image or no image
        List<Recognition> ringRecognitions = tfod.getRecognitions();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int prevSize = ringRecognitions.size();
        double start = om.runtime.time();
        if (tfod != null) {
            while (((om.runtime.time() - start) < timeToExit) && om.opModeIsActive()){ //loop for a set period of time

                ringRecognitions = tfod.getRecognitions();
                updatedRecognitions = tfod.getUpdatedRecognitions();
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.  Will be null if we're seeing the same thing so
                // we can use this to count loops with no change - might be hard to get "no change" even with robot being still
                // getRecognitions() will return the current stored recognitions which is what we want
                // if we want to output the image info

                om.telemetry.update();// update the telemetry to clear and report last loop
                // putting telemetry.update() here allows the last loop telemetry to be held and updated on exit

                if (ringRecognitions == null) { //not seeing any images
                    om.telemetry.addData("# Object Detected", 0);
                    imageType = "none";
                }
                else {
                    om.telemetry.addData("# Object Detected", ringRecognitions.size());

                    if(ringRecognitions.size() == 1) {
                        imageType = ringRecognitions.get(0).getLabel();// only update the value to the label if there's a single image
                    }
                    else if (ringRecognitions.size() == 0){
                        imageType = "None";
                    }
                    else {
                        imageType = "Multiple";
                    }

                    // step through the list of recognitions and display boundary info.
                    for (int i = 0; i < ringRecognitions.size(); i++) {
                        om.telemetry.addData(String.format("label (%d)", i), ringRecognitions.get(i).getLabel());
                        om.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
                        om.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());
                    }

                }
                loops +=1;
                om.telemetry.addData("Loop timer", " %1.3f s",om.runtime.time() - start);
                om.telemetry.addData("Loop counter", loops);

            }

            om.telemetry.addData("Image Type Returned", imageType);
            om.telemetry.addData("Loop counter", loops);
        }

        return imageType;//returns the last state observed
    }
}
