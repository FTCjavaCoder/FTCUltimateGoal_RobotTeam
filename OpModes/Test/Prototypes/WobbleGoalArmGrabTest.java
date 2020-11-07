package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Wobble Goal Arm and Grab Test", group="Test")

public class WobbleGoalArmGrabTest extends BasicTeleOp {

    public DcMotor wobbleGoalArm = null;
    public Servo wobbleGoalServo = null;

    double wobbleGoalPos = 0.5;// undecided values
    double wobbleGrabInc = 0.1;
    double wobbleGrabPos = 0.5;
    double wobbleReleasePos = 0;
    int wobbleArmTarget = 0;

    @Override
    public void runOpMode() {

//            initializeTeleOp();
        wobbleGoalArm = hardwareMap.get(DcMotor.class, "motor_wobble_goal");
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobble_goal_servo");

        wobbleGoalArm.setPower(0);
        wobbleGoalArm.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Initialized");
        telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                wobbleArmTarget += 100;
                wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            }
            if (gamepad1.dpad_down) {
                wobbleArmTarget -= 100;
                wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            }

            if (gamepad1.left_bumper) {
                wobbleGoalArm.setPower(0);
            }
            if (gamepad1.right_bumper) {
                wobbleGoalArm.setPower(0.1);
            }

            if (gamepad1.y) {

                wobbleGoalPos += wobbleGrabInc;
                TestGrabGoal(wobbleGoalPos);
                sleep(250);
            }
            if (gamepad1.a) {

                wobbleGoalPos -= wobbleGrabInc;
                TestGrabGoal(wobbleGoalPos);
                sleep(250);
            }

            telemetry.addData("Motor Variable", "Goal Arm Target (%.2f)", wobbleArmTarget);
            telemetry.addData("Motor Position", "Goal Arm Current Pos (%.2f)", wobbleGoalArm.getCurrentPosition());
            telemetry.addData("Servo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    wobbleGrabPos, wobbleReleasePos);
            telemetry.addData("Servo Position", "Servo Pos (%.2f)",
                    wobbleGoalPos);
            telemetry.update();
        }
    }

    public void TestGrabGoal(double armPos) {

        wobbleGoalServo.setPosition(armPos);// change to wobbleGoalServo
    }

}