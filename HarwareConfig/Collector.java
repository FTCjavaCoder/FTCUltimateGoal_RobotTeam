package UltimateGoal_RobotTeam.HarwareConfig;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Collector {

    /* Public OpMode members. */


    public Collector(BasicOpMode om, boolean tm)  {
        if(tm) {
            om.telemetry.addData("Status: ", "Called Constructor and TestMode is true");

        }
        else {
            om.telemetry.addData("Status: ", "Called Constructor and TestMode is false");
        }
    }
}
