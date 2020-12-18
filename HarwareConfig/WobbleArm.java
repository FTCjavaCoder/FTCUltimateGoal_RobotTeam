package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.Parameters.Constants;
import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class WobbleArm {

    /* Public OpMode members. */
    public DcMotor wobbleGoalArm    = null;
    public Servo   wobbleGoalServo  = null;

    Constants cons = new Constants();

    public double wobbleGoalPos = 0.5;// undecided values
    public double wobbleGrabInc = 0.1;
    public double wobbleGrabPos = 0.5;
    public double wobbleReleasePos = 0;
    public int wobbleArmTarget = 0;
    public int wobbleArmTargetAngle = 0;
    public int armDegInc = 1;
    public int armDegIncBig = 25;
    public double armPower = 0.25;
    public double powerInc = 0.05;
    public double armPowerHold = 0.7;
    public double armGearRatio = 2;

    public int c = 0; //temporary variable use to prevent us from choosing how we want to progress before we drop the wobble goal

    public WobbleArm(BasicOpMode om, boolean tm)  {
        if(tm) {
//            wobbleGoalServo = new Servo();
            om.telemetry.addData("ERROR: ", "Initializing DriveTrain in TestMode...");

        }
        else {
            wobbleGoalArm = om.hardwareMap.get(DcMotor.class, "motor_wobble_goal");
            wobbleGoalServo = om.hardwareMap.get(Servo.class, "wobble_goal_servo");

            wobbleGoalArm.setPower(0);
            wobbleGoalArm.setTargetPosition(0);
            wobbleGoalArm.setDirection(DcMotorSimple.Direction.FORWARD);
            wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }

    public void setWobbleMotorPower(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.left_bumper) {
            armPower -= powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }
        if (gamepad.right_bumper) {
            armPower += powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }

//        if (gamepad.b) {
//            armPower = 0.0;
//            wobbleGoalArm.setPower(armPower);
//            om.sleep(300);
//        }

    }

    public void changeWobbleMotorVariable(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_up) {
            wobbleArmTargetAngle += armDegInc;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }
        if (gamepad.dpad_down) {
            wobbleArmTargetAngle -= armDegInc;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }
        if (gamepad.x) {
            wobbleArmTargetAngle += armDegIncBig;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }
        if (gamepad.b) {
            wobbleArmTargetAngle -= armDegIncBig;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }

    }

    public void autoWobbleMotorVariable(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.y) {
            wobbleArmTargetAngle = 110;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }
        if (gamepad.a) {
            wobbleArmTargetAngle = 70;
            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
            om.sleep(300);
        }

        c = 1;

    }

    public void setWobbleMotorPosition(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_right) {
            wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            om.sleep(300);
        }

    }

    public void setWobbleServoPos(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.y) {

            wobbleGoalPos += wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }
        if (gamepad.a) {

            wobbleGoalPos -= wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }

    }

    public void pickUpWobble(BasicOpMode om) {




    }

    public void dropWobble(BasicOpMode om) {

        wobbleGoalArm.setPower(0.5);

        wobbleArmTargetAngle = 30;
        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
        wobbleGoalArm.setTargetPosition(wobbleArmTarget);

        wobbleGoalServo.setPosition(0.5);

        wobbleArmTargetAngle = 100;
        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (cons.DEGREES_TO_COUNTS_60_1 * armGearRatio));
        wobbleGoalArm.setTargetPosition(wobbleArmTarget);

        wobbleGoalServo.setPosition(0);

    }
}
