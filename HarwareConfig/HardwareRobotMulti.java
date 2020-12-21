package UltimateGoal_RobotTeam.HarwareConfig;


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