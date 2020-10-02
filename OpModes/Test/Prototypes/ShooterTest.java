package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;


@TeleOp(name="Shooter Test", group="Test")

public class ShooterTest extends BasicTeleOp {

    public double StoneArmPosBlue = 0.5;
    public double StoneArmPosRed = 0.5;
    public double CapstoneArmPos = 0.5;
    public double StoneArmPosIncrement = 0.05;

    @Override
    public void runOpMode() {

            initializeTeleOp();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.b) {

                StoneArmPosRed += StoneArmPosIncrement;
                TestServoRed(StoneArmPosRed);
                sleep(250);
            }
            if (gamepad1.x) {

                StoneArmPosRed -= StoneArmPosIncrement;
                TestServoRed(StoneArmPosRed);
                sleep(250);
            }

            telemetry.addLine("For Blue servo Y to increase position and A to decrease position");
            telemetry.addLine("For Red servo B to increase position and X to decrease position");
            telemetry.addData("Servo Variables", "Blue Command (%.2f), Red Command (%.2f)",
                    StoneArmPosBlue, StoneArmPosRed);
            telemetry.addData("Servo Positions", "Servo Blue (%.2f), Servo Red (%.2f)",
                    Billy.armServoBlue.getPosition(), Billy.armServoRed.getPosition());
            telemetry.update();
        }
    }

    public void TestServoRed(double armPos) {

        Billy.armServoRed.setPosition(armPos);
    }

    public void TestServoBlue(double armPos) {

        Billy.armServoBlue.setPosition(armPos);
    }

    public void TestServoCapstone(double armPos) {

        Billy.servoCapstoneRelease.setPosition(armPos);

    }

}