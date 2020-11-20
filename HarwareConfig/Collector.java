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

        }
        else {
            collectorWheel = om.hardwareMap.get(DcMotor.class, "motor_collector");

            collectorWheel.setPower(0);
            collectorWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void collectorControl(Gamepad g, BasicOpMode om) {

        if (g.dpad_left) {
            collectorPower = 0.0;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }

        if (g.dpad_up) {
            collectorPower += 0.10;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }
        if (g.dpad_down) {
            collectorPower -= 0.10;
            collectorWheel.setPower(collectorPower);
            om.sleep(300);
        }

    }
}
