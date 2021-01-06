package UltimateGoal_RobotTeam.HarwareConfig;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Arrays;
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
    private boolean[] configArrayHW = new boolean[]{ false, 	false, 	false, 		false, 		false,		false};//all defaults to false
    private final int TELEMETRY_MAX_SIZE = configArrayHW.length;//update so size tracks the max amount of HW
    private int telemetrySize = 1;
    private int telemetryActiveIndex = 0;
    private int[] telemetryOption = new int[TELEMETRY_MAX_SIZE];//used to determine telemetry to display, size must match TELEMETRY_MAX_SIZE

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
        // Use the array to make it easy to pass values but then values are decoded in constructor for ease of understanding
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
        telemetryOption[0] = 1;//ensure that that the initial telemetry window is active after initialization

    }

    /* Methods */
    //Consider the method to select the TM is here with robot, but the specific TM items reside with each HW class
    public enum telemetryDetails {BASIC,MAX}
    public void gamePadMultiTelemetry(BasicOpMode om, telemetryDetails details) {
        gamePadSetTelemetrySize(om);//sets number of windows from 1 to TELEMETRY_MAX_SIZE
        gamePadSetTelemetryIndex(om);//sets active index
        gamePadSetTelemetryOption(om);//sets telemetry (HW class item) option
        // Can add TeleOp unique items if desired before running multiTelemetry()
        multiTelemetry(om, details);//avoids duplicating the code for autonomous and TeleOp by using this method
        //multiTelemetry ends with om.telemetry.update() to refresh screen
    }
    public void gamePadSetTelemetryOption(BasicOpMode om) {
        if(om.gamepad1.back){
            telemetryOption[telemetryActiveIndex]  += 1;
            if(telemetryOption[telemetryActiveIndex]  > 6) {
                telemetryOption[telemetryActiveIndex]  = 0;//0 = inactive
            }
            om.sleep(300);
        }
    }
    public void gamePadSetTelemetrySize(BasicOpMode om) {
        if(om.gamepad1.left_bumper){
            telemetrySize  -= 1;
            if(telemetrySize < 1) {
                telemetrySize = 1;
            }
            om.sleep(300);
        }
        if(om.gamepad1.right_bumper){
            telemetrySize  += 1;
            if(telemetrySize > TELEMETRY_MAX_SIZE) {
                telemetrySize = TELEMETRY_MAX_SIZE;
            }
            om.sleep(300);
        }
    }
    public void gamePadSetTelemetryIndex(BasicOpMode om) {
        if(om.gamepad1.start){
            telemetryActiveIndex += 1;
            if(telemetryActiveIndex  >= TELEMETRY_MAX_SIZE) {
                telemetryActiveIndex = 0;
            }
            om.sleep(300);
        }
    }
    public void setTelemetryOption(int option) {
        telemetryOption[telemetryActiveIndex]  = option;
    }
    public void setTelemetrySize(int size) {
         telemetrySize  = size;
    }
    public void setTelemetryIndex(int index) {
         telemetryActiveIndex = index;
    }
    public void multiTelemetry(BasicOpMode om, telemetryDetails details) {
        // Uses the private global variables of telemetryOption[], telemetryActiveIndex, telemetrySize, TELEMETRY_MAX_SIZE
        // Some HW classes will have Basic and Max options (shooter during trial & error)
        //  - HashMap parameter to select option during robotUG constructor that sets an enum within HW class?

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
                    if(driveTrain != null) {driveTrain.getTelemetry(om);}
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;

                case 2 :
                    om.telemetry.addLine("WOBBLE GOAL (OPTION 2 - Back))...");
                    if(wobbleArm != null) {wobbleArm.getTelemetry(om);}
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;

                case 3 :
                    om.telemetry.addLine("SHOOTER (OPTION 3 - Back))...");
                    if(shooter != null) {
                        switch (details) {
                            case BASIC://basic telemetry option
                                shooter.getBasicTelemetry(om);//BASIC OPTION
                                break;
                            case MAX://MAX OPTION
                                shooter.getMaxTelemetry(om);//MAX OPTION for trial & error and troubleshooting
                        }
                    }
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;

                case 4 :
                    om.telemetry.addLine("CONVEYOR (OPTION 4 - Back))...");
                    if(conveyor != null) {conveyor.getTelemetry(om);}
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;
                case 5 :
                    om.telemetry.addLine("COLLECTOR (OPTION 5)...");
                    if(collector != null) {collector.getTelemetry(om);}
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;
                case 6 :
                    om.telemetry.addLine("IMAGE RECOGNITION (OPTION 6 - Back))...");
                    if(imageRecog != null){imageRecog.getTelemetry(om);}
                    else {om.telemetry.addLine("\t ... is NOT active!!");}
//                    om.telemetry.addLine("");//REMOVED added blank lines
                    break;
            }
            om.telemetry.addLine("");
        }
        om.telemetry.update();
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