package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import UltimateGoal_RobotTeam.HarwareConfig.DriveTrain;
import UltimateGoal_RobotTeam.OpModes.BasicOpMode;
//import OfflineCode.OfflineHW.Telemetry;
//import OfflineCode.Field.FieldConfiguration;

import UltimateGoal_RobotTeam.Utilities.PursuitLines;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="BasicAuto", group="Autonomous")
@Disabled
public class BasicAuto extends BasicOpMode {

    /**
     * configArray is arranged as
     * [0] = DriveTrain
     * [1] = Shooter
     * [2] = Conveyor
     * [3] = WobbleArm
     * [4] = Collector
     * [5] = ImageRecog
     * items that are 1 = true will be configured to the robot
     */
    // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
    boolean[] configArray = new boolean[]{true, true, true, true, true, true};

    // motor position must be integer number of counts
    public int forwardPosition = 0;//Target position in forward direction for drive motors
    public int rightPosition = 0;//Target position in right direction for drive motors
    public int clockwisePosition = 0;//Target position in clockwise direction for drive motors
    public int slideDistance = 0;

    public double extraFwd = 0;
    public double stoneSideways = 0;
    //    public double sideGrabStone = 11;// was 9 |||| was 18 {} decide how to make robo tmove forward and still grab stones
    public double foundationPosChange = 0;// 26 for unmoved Foundation, 0 for moved Foundation
    public double insideOutside = 0;// 0 for Inside, 24 for Outside
    public double foundationInOut = 0;// 0 for Inside, 26 for Outside
    public double foundationPush = 8;
    public int sideColor = 1;// + for Blue, - for Red, KEEP BLUE

    public double blockCamera = 0.525;
    public double extraFwdToBlock = 0;
    public boolean getSecondStone = true;

    public enum autoChoice {SkyStoneOutside, SkyStoneInside, SkyStoneOutsideUnmoved, SkyStoneInsideUnmoved}

    private static final float mmPerInch = 25.4f;

    public double stoneYLocation;

    //Define all double variables
    public double start = 0;//timer variable to use for setting waits in the code
    public float hsvValues[] = {0F, 0F, 0F};
    //    public String stonePos = "Unknown";
//
//    public boolean skystoneFound = false;
//
    public int ringSelect = 0;// 0 = no rings Wobble goes to zone A, 1 = 1 ring, Wobble goes to zone B, 4 = ringStack wobble goes to zone C
//
//    public double secondStoneBackup = 8;
//
//    public double extraRedDistance = 0;// added because of relative difference of starting position of robot on Red

    public double stoneArmUnderBridgeBlue;// for blue oriented servo
    public double stoneArmDownBlue;// was 0.05 and 0
    public double rackOutBlue;//

    public double stoneArmUnderBridgeRed;// for red oriented servo
    public double stoneArmDownRed;// was 0.7
    public double rackOutRed;// untested
//THIS IS JUST TO DECLARE THE VARIABLES YOU WILL ALSO NEED TO CHANGE THE VALUES IN THE INITIALIZE METHOD
//
//    public double vuforiaWaitTime = 1.5;// was 1
//
//    public double leftBound = 400;// in pixels defaults for TensorFlow
//    public double rightBound = 900;// in pixels defaults for TensorFlow

    public boolean drivingMiniBot = false;

    public double detectionRotateSpeed = 0.1;

    /* Coach Note: moved runTime to BasicOpMode so it would be available for all methods receiving om
     *
     */
//    public ElapsedTime runtime = new ElapsedTime(); //create a counter for elapsed time

    public boolean haveBlueRing = false;
    public boolean haveRedRing = false;
    public boolean haveBlueWobble1 = false;
    public boolean haveBlueWobble2 = false;
    public boolean haveRedWobble1 = false;
    public boolean haveRedWobble2 = false;

     //********** Added from OfflineOpModeLibs to BasicAuto forOfflineOpModeRunFile ******************
//    public FieldConfiguration fc = new FieldConfiguration();//NEEDED FOR OFFLINE

    public boolean writeBR = false;
    public boolean writeRR = false;
    public boolean writeBW1 = false;
    public boolean writeBW2 = false;
    public boolean writeRW1 = false;
    public boolean writeRW2 = false;
    public boolean robotSeeRing = false;
    public int IMUCounter =0;
    public final static int size = 300;
    public int[] collectorArray = new int[size];
    public int[] conveyorArray = new int[size];
    public int[] shooterArray = new int[size];
    public double[] wgaAngleArray = new double[size];
     //********** Added from OfflineOpModeLibs to BasicAuto forOfflineOpModeRunFile ******************


    /* Needed for Pure Pursuit */
    public ArrayList<PursuitPoint> fieldPoints = new ArrayList();
    public ArrayList<PursuitLines> lines = new ArrayList<>();

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;
    public boolean targetVisible = false;

    public VuforiaTrackables targetsSkyStone = null;

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public VuforiaTrackable stoneTarget = null;

//    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
//    public static final String LABEL_FIRST_ELEMENT = "Stone";
//    public static final String LABEL_SECOND_ELEMENT = "Skystone";
//
//    public TFObjectDetector tfod;
//
//    public double middleOfStone;
//
//    public double angleWhenFound;
//
//    public double zoneShift;
//
//    public double bridgeBack;

    @Override
    //    public void runOpMode() throws InterruptedException {
    public void runOpMode() {


    }
    @Override
    public void updateIMU() {
        //NEEDED FOR OFFLINE CODE TO UPDATE HW POSITIONS

        //Moved this method content to BasicAuto so that it is inherited in all BasicOpModes
        //Otherwise the OpModes run the blank BasicAuto code
        if (testModeActive) {
//            robotUG.driveTrain.imu.flCnt = robotUG.driveTrain.frontLeft.getCurrentPosition();
//            robotUG.driveTrain.imu.frCnt = robotUG.driveTrain.frontRight.getCurrentPosition();
//            robotUG.driveTrain.imu.brCnt = robotUG.driveTrain.backRight.getCurrentPosition();
//            robotUG.driveTrain.imu.blCnt = robotUG.driveTrain.backLeft.getCurrentPosition();

//            IMUCounter = robotUG.driveTrain.imu.counter;

            collectorArray[IMUCounter] = (int) Math.round(robotUG.collector.collectorPower);
            conveyorArray[IMUCounter] = (int) Math.round(robotUG.conveyor.conveyor_Power);
            shooterArray[IMUCounter] = (int) Math.round(robotUG.shooter.getShooter_Power());
            wgaAngleArray[IMUCounter] = robotUG.wobbleArm.getArmAngleDegrees() * Math.PI / 180.0;

//            fc.updateField(this);

            if (haveBlueRing) {
                writeBR = true;
            }
            if (haveRedRing) {
                writeRR = true;
            }
            if (haveBlueWobble1) {
                writeBW1 = true;
            }
            if (haveBlueWobble2) {
                writeBW2 = true;
            }
            if (haveRedWobble1) {
                writeRW1 = true;
            }
            if (haveRedWobble2) {
                writeRW2 = true;
            }

            try {
//
                if (IMUCounter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
                System.out.println(String.format("IMU Counter = %d", IMUCounter));
            }
        }
    }
    public void runCode() {


    }
    public void constructRobot() {


    }

     public void initialize() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        telemetry.addData(">", "Press Play to start");

        if (testModeActive) {

            readOrWriteHashMapOffline();
        } else {

            readOrWriteHashMap();
        }

        drivingMiniBot = false;
        //Values For Full Robot
//        detectionRotateSpeed = 0.1 * (40.0/60.0);
        cons.DEGREES_TO_COUNTS_40_1 = (1440.0 / 360.0) * (40.0 / 60.0);

//        stoneArmUnderBridgeBlue = 0.85;// for blue oriented servo
//        stoneArmDownBlue = 0.23;// for blue oriented servo was 0.20
//        rackOutBlue = 0.55;// was 0.75
//        stoneArmUnderBridgeRed = 0.20;// for red oriented servo
//        stoneArmDownRed = 0.80;// for red oriented servo untested was 0.77 untested
//        rackOutRed = 0.325;// untested was 0.25 also untested

        Billy.init(hardwareMap, testModeActive);

        //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
        Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Billy.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Reset all motor encoders
        Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Billy.frontLeft.setTargetPosition(0);
        Billy.frontRight.setTargetPosition(0);
        Billy.backLeft.setTargetPosition(0);
        Billy.shooterLeft.setTargetPosition(0);
        Billy.shooterRight.setTargetPosition(0);

        //Set all motors to position mode (assumes that all motors have encoders on them)
        Billy.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.shooterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Billy.frontLeft.setPower(0);
        Billy.frontRight.setPower(0);
        Billy.backLeft.setPower(0);
        Billy.backRight.setPower(0);
        Billy.shooterLeft.setPower(0);
        Billy.shooterRight.setPower(0);

        Billy.conveyorLeft.setPower(0);
        Billy.conveyorRight.setPower(0);

        if (testModeActive) {
            // DO NOTHING
        } else {

            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            allTrackables.add(targetsSkyStone.get(0));
        }

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)", forwardPosition, rightPosition, clockwisePosition);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();//Update telemetry to update display

    }// sets: RUN_TO_POSITION, ZeroPowerBehaviour.BRAKE, and 0 power & targetPos

    public void initializeMiniBot() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        telemetry.addData(">", "Press Play to start");

        if (testModeActive) {

            readOrWriteHashMapOffline();
        } else {

            readOrWriteHashMap();
        }

        drivingMiniBot = true;
        //Values For Mini Robot
        detectionRotateSpeed = 0.1;
        cons.DEGREES_TO_COUNTS_40_1 = (1440.0 / 360.0);

//        Billy.stoneArmInitBlue = 1;// for blue oriented servo was 0
//        Billy.stoneArmInitRed = 1;// for red oriented servo
//        stoneArmUnderBridgeBlue = 1;// for blue oriented servo was 0.25
//        stoneArmDownBlue = 0.4;// for blue oriented servo
//        stoneArmUnderBridgeRed = 0.75;// for red oriented servo
//        stoneArmDownRed = 0.3;// for red oriented servo (was 0.5, could adjust servo horn)

        Billy.initMiniBot(hardwareMap, testModeActive);

        //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
        Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset all motor encoders
        Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Billy.frontLeft.setTargetPosition(0);
        Billy.frontRight.setTargetPosition(0);
        Billy.backLeft.setTargetPosition(0);
        Billy.backRight.setTargetPosition(0);

        //Set all motors to position mode (assumes that all motors have encoders on them)
        Billy.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Billy.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Billy.frontLeft.setPower(0);
        Billy.frontRight.setPower(0);
        Billy.backLeft.setPower(0);
        Billy.backRight.setPower(0);
//
//        Billy.armServoBlue.setPosition(Billy.stoneArmInitBlue);
////        Billy.armServoRed.setPosition(stoneArmInitRed);
//
//        if (testModeActive) {
//            // DO NOTHING
//        }
//        else {
//
////            if (targetsSkyStone != null) {
//                targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//                stoneTarget = targetsSkyStone.get(0);
//                stoneTarget.setName("Stone Target");
//
//                allTrackables.add(targetsSkyStone.get(0));
////            }
//        }
        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)", forwardPosition, rightPosition, clockwisePosition);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();//Update telemetry to update display

    }
/* updateIMU() moved to BasicOpMode */


    public void forwardToViewRings() {
        // move forward ~18 inches

        robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack,20,-90,"Forward to rings",this);

//        // move forward ~18 inches pure pursuit
//        fieldPoints.add(new PursuitPoint(00,00));// no point yet
//        robotUG.driveTrain.drivePursuit(fieldPoints,this,"Forward to rings");

    }

    public void decideWobbleGoalZone(String ringsViewed) {

        switch (ringsViewed) {

            case "None":
                // Zone A pursuit points

                fieldPoints.add(new PursuitPoint(-54,-36));//sharper turn to avoid rings, keeping robot off center on tiles
                fieldPoints.add(new PursuitPoint(-54, -8));//more negative final location for wobble goal drop
            break;

            case "Single":
                // Zone B pursuit points


                fieldPoints.add(new PursuitPoint(-12,-36));//sharper turn to avoid rings
                fieldPoints.add(new PursuitPoint(-12,-12));
                fieldPoints.add(new PursuitPoint(-30,6));//keeping robot off center on tiles
                fieldPoints.add(new PursuitPoint(-30,16));//added straight section, more negative final location for wobble goal drop
            break;

            case "Quad":
                // Zone C pursuit points

                fieldPoints.add(new PursuitPoint(-54,-36));//sharper turn to avoid rings, keeping robot off center on tiles
                fieldPoints.add(new PursuitPoint(-54,40));//more negative final location for wobble goal drop
            break;

            case "Multiple":
                //el problemo
            break;

        }

    }

    public String testModeViewRings(){
        String seeRings;
        // Options are 0, 1, 4 rings on the field
        // ringSelect is global in BasicAuto and set within the OfflineOopMode for Field Configuration
        if(ringSelect == 1){
            seeRings = "Single";//Same string returned from ImageRecog.viewRingsTimed or viewRings
        }
        else if(ringSelect == 4){
            seeRings = "Quad";//Same string returned from ImageRecog.viewRingsTimed or viewRings
        }
        else {// if didn't see rings then return "None" to match ImageREcog
                seeRings = "None";
        }
        return seeRings;
    }
}