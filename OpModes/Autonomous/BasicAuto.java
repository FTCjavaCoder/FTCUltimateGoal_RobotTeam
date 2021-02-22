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

import UltimateGoal_RobotTeam.HarwareConfig.Conveyor;
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
    public final static int size = 3000;//Make sure that all arrays are large enough to go past 30 seconds - will set the size in Offline code
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

    public void interiorDriveToRings(double x, double y, double Wservo) {

        robotUG.wobbleArm.wobbleGoalServo.setPosition(Wservo);//this is a firm grip on the goal; 0.9
        telemetry.update();
        // Add points for Pure Pursuit motion - always start with where the robot was initialized to be on the field

        /* Drive to Wobble Goal and Scan the Number of Rings*/

        fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y)); // x: -33, y: -63; was -36
        fieldPoints.add(new PursuitPoint(x, y)); //was -36, -43; I changed x t0 -34.5; changed to -36, -41

        // Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
        for(int h=0;h<fieldPoints.size();h++) {
            telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
        }

        robotUG.driveTrain.drivePursuit(fieldPoints,this,"To View the Rings");
    }

    public void exteriorDriveToRings(double x1, double y1, double x2, double y2, double Wservo){

        robotUG.wobbleArm.wobbleGoalServo.setPosition(Wservo);//this is a firm grip on the goal
        telemetry.update();
        // Add points for Pure Pursuit motion - always start with where the robot was initialized to be on the field

        /* Drive to Wobble Goal and Scan the Number of Rings*/

        fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y)); //x: -57, y: -63
//		fieldPoints.add(new PursuitPoint(-57, -57));
        fieldPoints.add(new PursuitPoint(x1, y1));// WAS (-40, -46.2) updated to better view rings (changed it to -44, -35); 1/22: changed it back to -36, -52
        fieldPoints.add(new PursuitPoint(x2, y2)); // -37, -43

        // Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
        for(int h=0;h<fieldPoints.size();h++) {
            telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
        }

        robotUG.driveTrain.drivePursuit(fieldPoints,this,"To View the Rings");

        robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);

    }

    public String decideRingNumber() {

        String ringsViewed;//Define string for returning what rings were seen
        if(testModeActive){//Need this code for Offline
            /* THIS IS WHERE THE WAIT AND VIEW RINGS OCCURS*/
            int counts = 0;
            while(counts < 10) {// 10 counts or data points should equal 1 offline second (300 points = 30 s)
                telemetry.addLine("VIEWING RINGS");
                telemetry.addData("Counts", " %d", counts);
                telemetry.update();
                robotUG.driveTrain.robotNavigator(this);
                counts+=1;
            }
            ringsViewed = testModeViewRings();
            updateIMU();//Needs to be added 1 more time somewhere early in code -- missing a location where IMU angle is called but field isn't updated
        }
        else {//This is what runs on the robot
            double start = runtime.time();
            while ((runtime.time() - start) < 2.0) {
                // Do nothing but report TM for counter and wait for robot to settle before looking at rings
                robotUG.imageRecog.getTelemetry(this);
                telemetry.update();
            }
//		 	  ringsViewed = robotUG.imageRecog.viewRings(this, 25);//baseline method that runs for set number of loops
            ringsViewed = robotUG.imageRecog.viewRingsTimed(this, 0.5);// ALTERNATE method that runs based on time
        }

        telemetry.addLine("------------------------------------");
        telemetry.addData("Image Recognition Completed", "String Value: %s", ringsViewed);
        if(testModeActive){
            telemetry.update();//Offline code can't access gamepad or imageRecognition
        }
        else { // This is what runs on robot
//            pressAToContinue();
            robotUG.imageRecog.shutdown(); //shutdown after pressA to allow the driver to observe screen before moving on
        }

        return ringsViewed;
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

    public void driveToShoot(double x1, double y1, double x2, double y2, double shooterPwr) {

        fieldPoints.clear();// clear all the prior points
        fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));
        fieldPoints.add(new PursuitPoint(x1, y1));/* COACH CHANGED - for high goal - allow all options to align */ //is set to -48, -6
        fieldPoints.add(new PursuitPoint(x2, y2));/* COACH CHANGED - for high goal */ //is set to -30, -6

        /* Get Points for Drawing Lines in Visualization */
        fieldSimPoints();

        //TURN ON SHOOTER -- allow time to power up to full speed while driving
        robotUG.shooter.setShooter_Power(shooterPwr);//1.0 for high goal too much @ Y = -6, trying -8

        /* Drive to and Shoot the Power Shots */
        robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");
//		telemetry.addLine("Drive to Shooting Position");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 0.0, robotUG.driveTrain.robotHeading);
//		pressAToContinue();

        robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);/* COACH ADDED */

    }

    public void shootHighGoal(double collectorPwr, double time) {

        robotUG.conveyor.setMotion(Conveyor.motionType.UP);
        robotUG.collector.collectorWheel.setPower(collectorPwr);//need negative power to collector rings
        robotUG.collector.collectorPower = collectorPwr;//set variable to track in Offline code

        if(testModeActive){//accessing time will exceed size of data file and cause errors, run by number of counts
            int counts = 0;
            while(counts < 5 * time) {
                telemetry.addLine("Shoot High Goal x3");
                telemetry.addData("Counts", " %d", counts);
                telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.getShooter_Power());
                telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
                telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
                telemetry.update();
                robotUG.driveTrain.robotNavigator(this);
                counts+=1;
            }
        }
        else {
            double startTime = runtime.time();
            double shootTime = runtime.time() - startTime;
            while (shootTime < time) {//Since no sensors purely timed set of shots
                shootTime = runtime.time() - startTime;
                telemetry.addLine("Shoot high goal x 3");
                telemetry.addData("Timer", " %1.2f", shootTime);
                telemetry.addData("Shooter Power", "  %1.2f", robotUG.shooter.getShooter_Power());
                telemetry.addData("Conveyor Power", " %1.1f", robotUG.conveyor.conveyor_Power);
                telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
                telemetry.update();
            }
        }
        //TURN OFF CONVEYOR & COLLECTOR OFF
        robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
        robotUG.collector.collectorWheel.setPower(0.0);
        robotUG.collector.collectorPower = 0.0;//set variable to track in Offline code

    }

    public void shootPowerShot(double collectorPwr, double time) {

        shootHighGoal(collectorPwr,time);
        robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Left 7.5 In. to Shoot the Power Shot", this);

        shootHighGoal(collectorPwr,time);
        robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Left 7.5 In. to Shoot the Power Shot", this);

        shootHighGoal(collectorPwr,time);

        robotUG.shooter.shutdown();



    }

    public void fieldSimPoints() {

        if(testModeActive) {
            for (int h = 0; h < fieldPoints.size() - 1; h++) {
                lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h + 1).x, fieldPoints.get(h + 1).y));
            }
        }
    }
}