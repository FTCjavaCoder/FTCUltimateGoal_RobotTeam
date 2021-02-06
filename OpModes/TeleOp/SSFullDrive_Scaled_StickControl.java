package UltimateGoal_RobotTeam.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 *
 */

@TeleOp(name="FullDrive Scaled Stick Control", group="TeleOp")
@Disabled
public class SSFullDrive_Scaled_StickControl extends BasicTeleOp {

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
//            Billy.drivePowerAllLeftStickScaled(gamepad1, gamepad2);
            Billy.drivePowerAllLeftStickScaledSquared(gamepad1, gamepad2, this);

//            // use the left/right triggers on gamepad1 to rotate the robot counter/clockwise
//            Billy.rotatePowerRightStick(gamepad1, gamepad2);

            Billy.angleUnWrap();

            telemetry.addData("Status", "Run Time: ",runtime.toString());
            telemetry.addData("Robot Heading", "( %.2f )", Billy.robotHeading);
            telemetry.addData("Drive Motor Pos", "FL (%d), FR (%d), BR (%d), BL (%d)",
                    Billy.frontLeft.getCurrentPosition(), Billy.frontRight.getCurrentPosition(),
                    Billy.backRight.getCurrentPosition(), Billy.backLeft.getCurrentPosition());
            telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
                    forwardDirection, rightDirection, clockwise);
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(),
                    Billy.backRight.getPower());

            telemetry.update();

            idle();
        }
        stopMotors();

    } // main opMode end methods go below

}