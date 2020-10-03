package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;


@TeleOp(name="Wobble Goal Grab Test", group="Test")

public class WobbleGoalGrabTest extends BasicTeleOp {

    double wobbleGoalPos = 0.5;
    double wobbleGrabPos= 0.5;
    double wobbleReleasePos= 0.5;

    @Override
    public void runOpMode() {

            initializeTeleOp();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.y) {

                wobbleGoalPos = wobbleGrabPos;
                TestGrabGoal(wobbleGoalPos);
                sleep(250);
            }
            if (gamepad1.a) {

                wobbleGoalPos = wobbleReleasePos;
                TestGrabGoal(wobbleGoalPos);
                sleep(250);
            }

            telemetry.addLine("For Blue servo Y to increase position and A to decrease position");
            telemetry.addLine("For Red servo B to increase position and X to decrease position");
            telemetry.addData("Servo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    wobbleGrabPos, wobbleReleasePos);
            telemetry.addData("Servo Positions", "Servo Pos (%.2f)",
                    wobbleGoalPos);
            telemetry.update();
        }
    }

    public void TestGrabGoal(double armPos) {

        Billy.wobbleGoalServo.setPosition(armPos);// change to wobbleGoalServo
    }

}