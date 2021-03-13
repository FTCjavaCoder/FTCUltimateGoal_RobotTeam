package UltimateGoal_RobotTeam.HarwareConfig;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Collector {

    public double collectorPower = 0.0;
    public double servoPos = 0.0;
    private final double SERVO_INIT_POS = 1.0;//servo position with wheels up ready to start game, gets set in constructor

    /* Public OpMode members. */
    public DcMotor collectorWheel    = null;
    public Servo  collectorServo = null;

    public Collector(BasicOpMode om, boolean tm)  {
        if(tm) {
            om.telemetry.addData("Collector", " Initializing ...");
            om.telemetry.update();

//            collectorWheel = new DcMotor();collectorWheel.timeStep = om.timeStep;//NEEDED FOR OFFLINE

            collectorWheel.setPower(0);
            collectorWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();
        }
        else {
            om.telemetry.addData("Collector", " Initializing ...");
            om.telemetry.update();

            collectorWheel = om.hardwareMap.get(DcMotor.class, "motor_collector");
            collectorServo= om.hardwareMap.get(Servo.class, "servo_collector");

            collectorWheel.setPower(0);
            collectorWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            servoPos = SERVO_INIT_POS;
            collectorServo.setPosition(servoPos);

            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();
        }

    }

    public void collectorControl(Gamepad g, BasicOpMode om) {

        if (g.dpad_left) {
            collectorPower = 0.0;
            collectorWheel.setPower(collectorPower);
//            om.sleep(300);
        }

        // changed controls from gamepad up/down
        if (g.left_trigger > 0) {
            collectorPower += 0.10;
            collectorWheel.setPower(collectorPower);
//            om.sleep(300);
        }
        if (g.right_trigger > 0) {
            collectorPower -= 0.10;
            collectorWheel.setPower(collectorPower);
//            om.sleep(300);
        }

    }

    public void collectorOnOffControl(Gamepad g, BasicOpMode om) {

        if (g.left_trigger > 0) {
            collectorPower = 0;// collector off
            collectorWheel.setPower(collectorPower);
//            om.sleep(300);
        }
        if (g.right_trigger > 0) {
            collectorPower = -1;// collector on
            collectorWheel.setPower(collectorPower);
//            om.sleep(300);
        }

    }

    /* -- COACH NOTE: made a shutdown method for all hardware
     * - Added servo for collector assembly and setServoPos method
     */
    public void setServoPos(Gamepad gp, BasicOpMode om){
        // set initial controls - expecting to use GamePad2
        // Using dead zone of -0.2 to 0.2 to avoid multiple selections from motion out and back
        // may need separate method for preset positions
        if(om.runtime.time() >= om.timeLimitCollector) {

            if (gp.left_stick_y < -0.2) {//Y is negative away from driver
                servoPos += 0.1;
                collectorServo.setPosition(Range.clip(servoPos, 0, 0.7));
            }
            if (gp.left_stick_y > 0.2) {//Y is positive towards driver
                servoPos -= 0.1;
                collectorServo.setPosition(Range.clip(servoPos, 0, 0.7));
            }
            if (gp.right_stick_y < -0.2) {//Y is negative away from driver
                servoPos += 0.05;
                collectorServo.setPosition(Range.clip(servoPos, 0, 0.7));
            }
            if (gp.right_stick_y > 0.2) {//Y is positive towards driver
                servoPos -= 0.05;
                collectorServo.setPosition(Range.clip(servoPos, 0, 0.7));
            }

            om.timeLimitCollector = om.runtime.time() + 0.3;
        }

    }
    public void getTelemetry(BasicOpMode  om){
        om.telemetry.addData("\tMotor Power", "Command =%.2f, Actual= %.2f", collectorPower, collectorWheel.getPower());
        om.telemetry.addData("\tServo Position", " %.2f", collectorServo.getPosition());
        // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
        // Can add more lines as needed or max a BASIC and MAX TM option
    }
    public void shutdown(){
        collectorWheel.setPower(0.0);
    }

}
