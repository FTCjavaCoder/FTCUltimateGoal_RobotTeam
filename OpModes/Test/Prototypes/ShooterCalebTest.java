package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Shooter Test Caleb", group="Test")

public class ShooterCalebTest extends BasicTeleOp {

    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;

    @Override
    public void runOpMode() {

        shooterLeft = hardwareMap.get(DcMotor.class, "motor_shooterL");
        shooterRight = hardwareMap.get(DcMotor.class, "motor_shooterR");

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            shooterLeft.setPower(-gamepad1.left_stick_y);
            shooterRight.setPower(gamepad1.left_stick_y);

            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", shooterRight.getPower(), shooterLeft.getPower());
            telemetry.update();
        }
    }
}