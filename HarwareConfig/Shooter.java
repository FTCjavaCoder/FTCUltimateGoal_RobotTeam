package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Shooter {

    public double shooter_Power = 0;//KS can be private - not accessed outside class

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
            om.telemetry.addData("ERROR: ", "Initializing Shooter in TestMode...");
            om.telemetry.update();

        }
        else {
            om.telemetry.addData("Shooter", " Initializing ...");
            om.telemetry.update();

            shooterLeft =om.hardwareMap.get(DcMotor .class,"motor_shooterL");
            shooterRight =om.hardwareMap.get(DcMotor .class,"motor_shooterR");

            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();
        }
    }

    public void ShooterControl(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_left) {
            shooter_Power = 0;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(shooter_Power);
            om.sleep(300);
        }

        if (gamepad.dpad_up) {
            shooter_Power -= 0.05;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(-shooter_Power);
            om.sleep(300);
        }
        if (gamepad.dpad_down) {
            shooter_Power += 0.05;
            shooterLeft.setPower(shooter_Power);
            shooterRight.setPower(-shooter_Power);
            om.sleep(300);
        }

    }
    /* -- COACH NOTE: need to add method for autonomous shooter control
     *  - method required to set power like "dpad" button
     *  - also need a stop method to shutdown after 3 shots
     *  - made a shutdown method for all hardware
     *
     */
    public void shutdown(){
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }
}
