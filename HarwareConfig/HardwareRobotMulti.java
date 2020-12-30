package UltimateGoal_RobotTeam.HarwareConfig;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;


/**
 * NEW FOR ULTIMATE GOAL - Coach File - can be used by team or HardwareRobot can still be used
 * 11/1/20 - this file now configures a robot from the base hardware elements
 * Enables same config that is developed for bench test to be used for final robot
 * Hardware control methods are located with the individual HW elements
 * HW Elements - DriveTrain, Shooter, Conveyor, WobbleArm, Collector (item to collect rings)
 * Eliminates repeat code for MiniBot and MainBot
 * Enables building multiple desired configs from working code
 * Example MiniBot  = Robot.DriveTrain only
 * Example Current Bot = Robot.DriveTrain + .Shooter + .Conveyor + .WobbleArm
 * DriveTrain will contain IMU and navigation
 * Approach should push the testMode determination for offline code down to the HW elements
 */
public class HardwareRobotMulti
{
    /* Public OpMode members. */

    public DriveTrain driveTrain   = null;
    public Shooter shooter   = null;
    public Conveyor conveyor   = null;
    public WobbleArm wobbleArm   = null;
    public Collector collector   = null;
    public ImageRecog imageRecog   = null;
    /* Deleting unused items - HardwareMap is in OpMode and is passed anyways, period isn't used
    public HardwareMap hwMap           =  null;
    public ElapsedTime period  = new ElapsedTime();
    */ // End Delete
// HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector	ImageRecog
    boolean[] configArrayHW = new boolean[]{ false, 	false, 	false, 		false, 		false,		true};//all defaults to false
    private final int TELEMETRY_MAX_SIZE = 3;
    private int telemetrySize = 1;
    private int telemetryActiveIndex = 0;
    private int[] telemetryOption = new int[]{1,0,0};//used to determine telemetry to display, must match TELEMETRY_MAX_SIZE

    /** Constructor
     * This constructs the robot needed for each OpMode and can be called in that OpMode to complete the null robotUG
     * @param om - the OpMode
     * @param configArray - boolean array for setting up the robot - true for items includes
     *  configArray is arranged as
     * [0] = DriveTrain
     * [1] = Shooter
     * [2] = Conveyor
     * [3] = WobbleArm
     * [4] = Collector
     * [5] = ImageRecog
     *  items that are 1 = true will be configured to the robot
     *
     * @param tm : boolean for whether testModeActive is true or false - true calls the testmode version of the constructor
     */
    public HardwareRobotMulti(BasicOpMode om, boolean[] configArray, boolean tm){
        //configArray has True or False values for each subsystem HW element
        // Use the array to make it easy to pass values bu then values are decoded in constructor for ease of understanding
        configArrayHW = configArray;
        boolean trueDriveTrain = configArray[0];
        boolean trueShooter = configArray[1];
        boolean trueConveyor= configArray[2];
        boolean trueWobbleArm = configArray[3];
        boolean trueCollector = configArray[4];
        boolean trueImageRecog = configArray[5];


        if (trueDriveTrain) {
            driveTrain = new DriveTrain(om,tm);
        }
        if (trueShooter) {
            shooter = new Shooter(om,tm);
        }
        if (trueConveyor) {
            conveyor = new Conveyor(om,tm);
        }
        if (trueWobbleArm) {
            wobbleArm = new WobbleArm(om,tm);
        }
        if (trueCollector) {
            collector = new Collector(om,tm);
        }
        if (trueImageRecog) {
            imageRecog = new ImageRecog(om,tm);
        }

    }

    /* Methods */

    public void multiTelemetry(BasicOpMode om) {
        setTelemetrySize(om);
        setTelemetryIndex(om);
        setTelemetryOption(om);
        om.telemetry.addData("Run Time", " %s",om.runtime.toString());
        om.telemetry.addData("Telemetry Size:", "%d (Bumpers),  Active TM window: %d (Start)",telemetrySize, telemetryActiveIndex);
        om.telemetry.addLine("--------------------------------------");

        for(int j =0; j < telemetrySize; j++){
            switch (telemetryOption[j]) {

                case 0 :
                    om.telemetry.addData("OPTION 0", "TM Window %d NOT Active (Back)",j);
                    break;
                case 1 :
                    om.telemetry.addLine("DRIVE TRAIN (OPTION 1 - Back)...");
                    if(driveTrain != null) {
                        om.telemetry.addLine("ROBOT GAMEPAD COMMANDS ...");
                        om.telemetry.addData("\tCommands", "FWD (%.2f), Right (%.2f), CW (%.2f)",
                                driveTrain.forwardDirection, driveTrain.rightDirection, driveTrain.clockwise);
                        om.telemetry.addData("\tDrive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                                driveTrain.frontLeft.getPower(), driveTrain.frontRight.getPower(), driveTrain.backLeft.getPower(),
                                driveTrain.backRight.getPower());
                        om.telemetry.addLine("ROBOT LOCATION ...");
                        om.telemetry.addData("\tRobot Field Position (X, Y)", " ( %1.1f, %1.1f)", driveTrain.robotFieldLocation.x, driveTrain.robotFieldLocation.y);
                        om.telemetry.addData("\tRobot Angles", " \t Heading: %1.1f, \t Field: %1.1f", driveTrain.robotHeading, driveTrain.robotFieldLocation.theta);
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;

                case 2 :
                    om.telemetry.addLine("WOBBLE GOAL (OPTION 2 - Back))...");
                    if(wobbleArm != null) {
                        om.telemetry.addData("\tArm Target", "Goal Arm Target Angle (%d) degrees", wobbleArm.wobbleArmTargetAngle);
                        om.telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",wobbleArm.getArmAngleDegrees());
                        om.telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", wobbleArm.armPower, wobbleArm.wobbleGoalArm.getTargetPosition());
                        om.telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", wobbleArm.wobbleGoalArm.getCurrentPosition());
                        om.telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                                wobbleArm.wobbleGrabPos, wobbleArm.wobbleReleasePos);
                        om.telemetry.addData("\tServo Position", "Servo Pos (%.2f)",wobbleArm.wobbleGoalServo.getPosition());
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;

                case 3 :
                    om.telemetry.addLine("SHOOTER (OPTION 3 - Back))...");
                    if(shooter != null) {
                        om.telemetry.addData("\tMotor Power", "Right Power (%.2f), Left Power (%.2f)", shooter.shooterRight.getPower(), shooter.shooterLeft.getPower());
                        om.telemetry.addData("\tShooter Power Variable", "Variable: Shooter Power (%.2f)", shooter.shooter_Power);
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;

                case 4 :
                    om.telemetry.addLine("CONVEYOR (OPTION 4 - Back))...");
                    if(conveyor != null) {
                        om.telemetry.addData("\tServo Power", "Right Power (%.2f), Left Power (%.2f)", conveyor.conveyorRight.getPower(), conveyor.conveyorLeft.getPower());
                        om.telemetry.addData("\tConveyor Power Variable", "Variable: Conveyor Power (%.2f)", conveyor.conveyor_Power);
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;
                case 5 :
                    om.telemetry.addLine("COLLECTOR (OPTION 5)...");
                    if(collector != null) {
                        om.telemetry.addData("\tCommands", "collectorPower (%.2f), collectorWheel Power (%.2f)", collector.collectorPower, collector.collectorWheel.getPower());
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;
                case 6 :
                    om.telemetry.addLine("IMAGE RECOGNITION (OPTION 6 - Back))...");
                    if(imageRecog != null){
                        List<Recognition> ringRecognitions = imageRecog.tfod.getRecognitions();

                        for (int i = 0; i < ringRecognitions.size(); i++) {
                            om.telemetry.addData(String.format("\tLabel (%d)", i), ringRecognitions.get(i).getLabel());
                            om.telemetry.addData(String.format("\t\tleft,top (%d)", i), "%.03f , %.03f",
                                    ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
                            om.telemetry.addData(String.format("\t\tright,bottom (%d)", i), "%.03f , %.03f",
                                    ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());
                        }
                    }
                    else {
                        om.telemetry.addLine("\t ... is NOT active!!");
                    }
                    om.telemetry.addLine("");
                    break;
            }
            om.telemetry.addLine("");
        }
        om.telemetry.update();
    }
    public void setTelemetryOption(BasicOpMode om) {
        if(om.gamepad1.back){
            telemetryOption[telemetryActiveIndex]  += 1;
            if(telemetryOption[telemetryActiveIndex]  > 6) {
                telemetryOption[telemetryActiveIndex]  = 0;
            }
            om.sleep(300);// is having the sleep() necessary when this is not immediately in a loop?
        }
    }
    public void setTelemetrySize(BasicOpMode om) {
        if(om.gamepad1.left_bumper){
            telemetrySize  -= 1;
            if(telemetrySize < 1) {
                telemetrySize = 1;
            }
            om.sleep(300);// is having the sleep() necessary when this is not immediately in a loop?
        }
        if(om.gamepad1.right_bumper){
            telemetrySize  += 1;
            if(telemetrySize > TELEMETRY_MAX_SIZE) {
                telemetrySize = TELEMETRY_MAX_SIZE;
            }
            om.sleep(300);// is having the sleep() necessary when this is not immediately in a loop?
        }
    }
    public void setTelemetryIndex(BasicOpMode om) {
        if(om.gamepad1.start){
            telemetryActiveIndex += 1;
            if(telemetryActiveIndex  >= TELEMETRY_MAX_SIZE) {
                telemetryActiveIndex = 0;
            }
            om.sleep(300);// is having the sleep() necessary when this is not immediately in a loop?
        }
    }

    public void shutdownAll(){
        if (configArrayHW[0]) {
            driveTrain.shutdown();
        }
        if (configArrayHW[1]) {
            shooter.shutdown();
        }
        if (configArrayHW[2]) {
            conveyor.shutdown();
        }
        if (configArrayHW[3]) {
            wobbleArm.shutdown();
        }
        if (configArrayHW[4]) {
            collector.shutdown();
        }
        if (configArrayHW[5]) {
            imageRecog.shutdown();
        }
    }
}