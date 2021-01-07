package UltimateGoal_RobotTeam.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobot;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Parameters.Constants;

public class BasicOpMode extends LinearOpMode {

    public HardwareRobot Billy = new HardwareRobot();// call using Billy.(for hardware or angle unwrap method)
    public HardwareRobotMulti robotUG = null; // Adds new multi-HW element robot to OpMode
    // Need to configure robotUG in specific OpModes that use it
    public Constants cons = new Constants();// call using cons.(constant DRIVE_POWER_LIMIT etc.)

    //********************UPDATED 12/27/19 for OpMpde HashMap *********************************
//    public OpModeParamFunctions ompf = new OpModeParamFunctions();
    /* UPDATED TO BE PRIVATE */
    private boolean loadFile = true;
    public String fileName = "AndroidHashMapFile.txt";
    public String fileNameEdited = "AndroidHashMapFileEdited.txt";
    //********************UPDATED 12/27/19 for OpMpde HashMap *********************************

    public boolean fileWasRead = true;
    public String hashMapFile = "HashMapFile.txt";
    public String autoOptionFile = "AutoOptionFile.txt";
    public int selected = 0;

    public double DeltaH = 0;
    public double currentH = 0;

    public boolean testModeActive = false;

    public ElapsedTime runtime = new ElapsedTime(); //create a counter for elapsed time

//    public Telemetry telemetry = new Telemetry();//used for OfflineCode

    public BasicOpMode() {

    }

    @Override
    public void runOpMode() {

    }

    public void pressAToContinue() {

        telemetry.addLine("**********************");
        telemetry.addLine("Press A to continue");
        telemetry.update();
        while (!gamepad1.a && opModeIsActive()) {

            idle();
        }
        sleep(300);
    }

    /* COACH NOTE: updated readOrWriteHashMap methods to receive boolean as output from Constants methods
     *   - USING PRIVATE VARIABLE IN OPMODE THAT IS SET BY METHOD IN CONSTANTS
     *   - This makes it easier to see where the private variable is set
     */
    public void readOrWriteHashMap() {

        fileWasRead = cons.readFromPhone(hashMapFile, this);
        telemetry.addData("Existing File Was Read?","%s", fileWasRead);

        if (!fileWasRead) {

            cons.defineParameters();
            cons.writeToPhone(hashMapFile, this);

            fileWasRead = cons.readFromPhone(hashMapFile, this);//Can be eliminated because HashMap exists in constants from defineParameters
            telemetry.addData("Created File, File Was Read?","%s", fileWasRead);
            //No telemetry.update();
            //Note differences in Pancho's Constants_Pw
        }
        telemetry.update();
        cons.initParameters();
    }

    public void readOrWriteHashMapOffline() {

        fileWasRead = cons.readFromFile(hashMapFile, this);
        telemetry.addData("Existing File Was Read?","%s", fileWasRead);
        telemetry.update();
        if (!fileWasRead) {

            cons.defineParameters();
            cons.writeToFile(hashMapFile, this);

            fileWasRead = cons.readFromFile(hashMapFile, this);//Can be eliminated because HashMap exists in constants from defineParameters
            telemetry.addData("Created File, File Was Read?","%s", fileWasRead);
            telemetry.update();
        }
        // ******************** ADDED initPARAMETERS ****************
        cons.initParameters();
        //Note differences in Pancho's Constants_Pw implementation
        // ******************** ADDED initPARAMETERS ****************

    }

//    public void readOrWriteHashMapAO() {
//
//        cons.readFromPhoneAO(hashMapFile, this);
//        telemetry.addData("Existing File Was Read?","%s", fileWasRead);
//
//        if (!fileWasRead) {
//
//            cons.defineAutoOptions();
//            cons.writeToPhoneAO(hashMapFile, this);
//
//            cons.readFromPhoneAO(hashMapFile, this);
//            telemetry.addData("Created File, File Was Read?","%s", fileWasRead);
//        }
//    }

}