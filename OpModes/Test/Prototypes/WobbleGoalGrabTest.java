package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;


@TeleOp(name="Wobble Goal Grab Test", group="Test")
@Disabled // Initialize Servos properly
public class WobbleGoalGrabTest extends BasicTeleOp {

    public Servo wobbleGoalServo = null;
    
    double wobbleGoalPos = 0.5;// undecided values
    double wobbleGrabPos= 0.5;
    double wobbleReleasePos= 0;

    @Override
    public void runOpMode() {

//            initializeTeleOp();
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobble_goal_servo");

        telemetry.addLine("Initialized");
        telemetry.update();
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

        wobbleGoalServo.setPosition(armPos);// change to wobbleGoalServo
    }

}