package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;


@TeleOp(name="Shooter Test Caleb", group="Test")

public class ShooterCalebTest extends BasicTeleOp {

    @Override
    public void runOpMode() {

            initializeTeleOp();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.left_stick_y < -0.25) {

                Billy.shooterLeft.setPower(cons.SHOOTER_POWER_LIMIT);// parameter starts at 0.5
                Billy.shooterRight.setPower(cons.SHOOTER_POWER_LIMIT);// parameter starts at 0.5
            }
            if (gamepad1.left_stick_y >= -0.25) {

                Billy.shooterLeft.setPower(0);
                Billy.shooterRight.setPower(0);
            }

            telemetry.addLine("For Blue servo Y to increase position and A to decrease position");
            telemetry.addLine("For Red servo B to increase position and X to decrease position");
            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", Billy.shooterRight.getPower(), Billy.shooterLeft.getPower());
            telemetry.update();
        }
    }

}