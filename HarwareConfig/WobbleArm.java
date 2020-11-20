package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class WobbleArm {

    /* Public OpMode members. */
    public DcMotor wobbleGoalArm    = null;
    public Servo   wobbleGoalServo  = null;

    public double wobbleGoalPos = 0.5;// undecided values
    public double wobbleGrabInc = 0.1;
    public double wobbleGrabPos = 0.5;
    public double wobbleReleasePos = 0;
    public int wobbleArmTarget = 0;
    public int armPosInc = 10;
    public double armPower = 0.25;
    public double powerInc = 0.05;
    public double wobbleArmPower = 0;
    public double armPowerHold = 0.7;

    public WobbleArm(BasicOpMode om, boolean tm)  {
        if(tm) {
//            wobbleGoalServo = new Servo();
            om.telemetry.addData("ERROR: ", "Initializing DriveTrain in TestMode...");

        }
        else {
            wobbleGoalArm = om.hardwareMap.get(DcMotor.class, "motor_wobble_goal");
            wobbleGoalServo = om.hardwareMap.get(Servo.class, "wobble_goal_servo");

            wobbleGoalArm.setPower(0);
            wobbleGoalArm.setDirection(DcMotorSimple.Direction.FORWARD);
            wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }

    public void setWobbleMotorPower(Gamepad gamepad1, BasicOpMode om) {

        if (gamepad1.left_bumper) {
            armPower -= powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }
        if (gamepad1.right_bumper) {
            armPower += powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }

        if (gamepad1.b) {
            armPower = 0;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }

    }

    public void changeWobbleMotorVariable(Gamepad gamepad1, BasicOpMode om) {

        if (gamepad1.dpad_up) {
            wobbleArmTarget += armPosInc;
            om.sleep(300);
        }
        if (gamepad1.dpad_down) {
            wobbleArmTarget -= armPosInc;
            om.sleep(300);
        }

    }

    public void setWobbleMotorPosition(Gamepad gamepad1, BasicOpMode om) {

        if (gamepad1.dpad_right) {
            wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            om.sleep(300);
        }

    }

    public void wobbleArmStickControl(Gamepad gamepad2, BasicOpMode om) {

        if(gamepad2.left_stick_y != 0) {

            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            wobbleArmPower = -gamepad2.left_stick_y;
            wobbleGoalArm.setPower(wobbleArmPower);
        }
        else {
//            wobbleGoalArm.setPower(0);
//            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            wobbleArmTarget = wobbleGoalArm.getCurrentPosition();
//            wobbleGoalArm.setTargetPosition(wobbleArmTarget);
//            wobbleGoalArm.setPower(armPowerHold);
        }

    }

    public void setWobbleServoPos(Gamepad gamepad2, BasicOpMode om) {

        if (gamepad2.y) {

            wobbleGoalPos += wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }
        if (gamepad2.a) {

            wobbleGoalPos -= wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }

    }

}
