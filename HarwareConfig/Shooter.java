package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Shooter {

    public double shooter_Power = 0;

    /* Public OpMode members. */
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;

    /**
     * CONSTRUCTORS
     * Taking the approach that when constructing robot all the initialization items can be run
     * Approach is that null Robot existing in BasicOpMode and then each OpMode constructs the Robot it needs
     * Methods within BasicAuto to be re-used would need to pass the Robot to have the correct robot config
     * This enables having similar code for different configs
     *
     * @param om: this is the OpMode that is constructing the
     * @param tm: this is the testMode boolean, if in testMode (Offline) then need to create new instances
     *            if NOT in testMode (real robot) need to map null objects
     *            Believe this can be made such that testMode robot is constructed on it's own and eliminates this boolean
     *            initTestMode is a separate constructor
     */

    public Shooter(BasicOpMode om, boolean tm)  {
        if(tm) {

//            shooterLeft = new DcMotor();
//            shooterRight = new DcMotor();
            om.telemetry.addData("ERROR: ", "Initializing DriveTrain in TestMode...");

        }
        else {

            shooterLeft =om.hardwareMap.get(DcMotor .class,"motor_shooterL");
            shooterRight =om.hardwareMap.get(DcMotor .class,"motor_shooterR");
        }
    }

    public void ShooterControl(Gamepad g1, BasicOpMode om) {

        if (g1.dpad_left) {
            shooter_Power = 0;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(shooter_Power);
            om.sleep(300);
        }

        if (g1.dpad_up) {
            shooter_Power -= 0.05;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(-shooter_Power);
            om.sleep(300);
        }
        if (g1.dpad_down) {
            shooter_Power += 0.05;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(-shooter_Power);
            om.sleep(300);
        }

    }
}
