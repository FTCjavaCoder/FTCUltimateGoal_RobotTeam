package UltimateGoal_RobotTeam.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 *
 */

@TeleOp(name="Conveyor & Shooter + Drive", group="TeleOp")

public class ConveyorShooterDrive extends BasicTeleOp {

    @Override
    public void runOpMode() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        initializeTeleOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Billy.initIMU(this);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Set Drive Motor Power
            Billy.drivePowerAll(gamepad1, gamepad2,this);

            if (gamepad1.dpad_left) {
                shooter_Power = 0;
                Billy.shooterLeft.setPower(shooter_Power);
                Billy.shooterRight.setPower(shooter_Power);
                sleep(300);
            }

            if (gamepad1.dpad_up) {
                shooter_Power -= 0.05;
                Billy.shooterLeft.setPower(shooter_Power);
                Billy.shooterRight.setPower(-shooter_Power);
                sleep(300);
            }
            if (gamepad1.dpad_down) {
                shooter_Power += 0.05;
                Billy.shooterLeft.setPower(shooter_Power);
                Billy.shooterRight.setPower(-shooter_Power);
                sleep(300);
            }

            if (gamepad1.x) {
                conveyor_Power = 1;
                Billy.conveyorLeft.setPower(conveyor_Power);
                Billy.conveyorRight.setPower(-conveyor_Power);
                sleep(300);
            }
            if (gamepad1.y) {
                conveyor_Power = -1;
                Billy.conveyorLeft.setPower(conveyor_Power);
                Billy.conveyorRight.setPower(-conveyor_Power);
                sleep(300);
            }
            if (gamepad1.b) {
                conveyor_Power = 0;
                Billy. conveyorLeft.setPower(conveyor_Power);
                Billy.conveyorRight.setPower(conveyor_Power);
                sleep(300);
            }

            Billy.angleUnWrap();

            telemetry.addData("Status", "Run Time: ",runtime.toString());
            telemetry.addData("Robot Heading", "( %.2f )", Billy.robotHeading);
            telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
                    forwardDirection, rightDirection, clockwise);
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(),
                    Billy.backRight.getPower());
            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", Billy.shooterRight.getPower(), Billy.shooterLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", shooter_Power);
            telemetry.addData("Servo Power", "Right Power (%.2f), Left Power (%.2f)", Billy.conveyorRight.getPower(), Billy.conveyorLeft.getPower());
            telemetry.addData("Conveyor Power Variable", "Variable: Shooter Power (%.2f)", conveyor_Power);
            telemetry.update();

            idle();
        }
        stopMotors();

    } // main opMode end methods go below

}