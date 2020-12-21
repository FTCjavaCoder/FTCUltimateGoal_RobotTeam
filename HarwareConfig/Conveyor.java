package UltimateGoal_RobotTeam.HarwareConfig;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

public class Conveyor {

    /* Public Variables */
    public double conveyor_Power = 0;//KS: can be private and final ... a constant

    /* Public OpMode members. */
    public CRServo conveyorLeft = null;
    public CRServo conveyorRight = null;

    public Conveyor(BasicOpMode om, boolean tm) {

        if(tm){
//            conveyorLeft = new CRServo();
////            conveyorRight = new CRServo();
            om.telemetry.addData("ERROR: ", "Initializing Conveyor in TestMode...");
            om.telemetry.update();

        }
        else{
            om.telemetry.addLine("... ");//add line space
            om.telemetry.addData("Status: ", "Initializing Conveyor ...");
            om.telemetry.update();
            conveyorLeft = om.hardwareMap.get(CRServo.class, "servo_conveyorL");
            conveyorRight = om.hardwareMap.get(CRServo.class, "servo_conveyorR");

            om.telemetry.addLine(" ");//blank line space
            om.telemetry.addData("Status: ", "Conveyor Initialized");
            om.telemetry.update();

        }
    }

    public void ConveyorControl(Gamepad gamepad, BasicTeleOp om) {

        if (gamepad.a) {
            conveyor_Power = 1;
            conveyorLeft.setPower(conveyor_Power);
            conveyorRight.setPower(-conveyor_Power);
            om.sleep(300);
        }
        if (gamepad.y) {
            conveyor_Power = -1;
            conveyorLeft.setPower(conveyor_Power);
            conveyorRight.setPower(-conveyor_Power);
            om.sleep(300);
        }
        if (gamepad.b) {
            conveyor_Power = 0;
            conveyorLeft.setPower(conveyor_Power);
            conveyorRight.setPower(conveyor_Power);
            om.sleep(300);
        }

    }
    /* -- COACH NOTE: need to add method for autonomous conveyor control
     * -- method required to set power like "y" button
     * -- also need a stop method to keep rings from continually advancing
     *  - made a shutdown method for all hardware
     *
     */
    public void shutdown(){
        conveyorLeft.setPower(0.0);
        conveyorLeft.setPower(0.0);
    }
}
