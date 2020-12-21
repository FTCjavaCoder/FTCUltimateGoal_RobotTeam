package UltimateGoal_RobotTeam.HarwareConfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Collector {

    public double collectorPower = 0.0;

    /* Public OpMode members. */
    public DcMotor collectorWheel    = null;


    public Collector(BasicOpMode om, boolean tm)  {
        if(tm) {
//            wobbleGoalServo = new Servo();
            om.telemetry.addData("ERROR: ", "Initializing Collector in TestMode...");
            om.telemetry.update();
        }
        else {
            om.telemetry.addLine("... ");//add line space
            om.telemetry.addData("Status: ", "Initializing Collector ...");
            om.telemetry.update();

            collectorWheel = om.hardwareMap.get(DcMotor.class, "motor_collector");

            collectorWheel.setPower(0);
            collectorWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            om.telemetry.addLine(" ");//blank line space
            om.telemetry.addData("Status: ", "Collector Initialized");
            om.telemetry.update();
        }

    }

    public void collectorControl(Gamepad g, BasicOpMode om) {

        if (g.dpad_left) {
            collectorPower = 0.0;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }

        // changed controls from gamepad up/down
        if (g.left_trigger > 0) {
            collectorPower += 0.10;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }
        if (g.right_trigger > 0) {
            collectorPower -= 0.10;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }

    }
    /* -- COACH NOTE: made a shutdown method for all hardware
     *
     */
    public void shutdown(){
        collectorWheel.setPower(0.0);
    }

}
