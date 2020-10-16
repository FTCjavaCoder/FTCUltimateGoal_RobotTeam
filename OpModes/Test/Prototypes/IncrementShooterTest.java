package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Increment Shooter Test", group="Test")

public class IncrementShooterTest extends BasicTeleOp {

    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;

    public double shooter_Power = 0;

    @Override
    public void runOpMode() {

        shooterLeft = hardwareMap.get(DcMotor.class, "motor_shooterL");
        shooterRight = hardwareMap.get(DcMotor.class, "motor_shooterR");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

//        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.b) {
                shooter_Power = 0;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(shooter_Power);
                sleep(500);
            }

            if (gamepad1.dpad_up) {
                shooter_Power -= 0.05;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(-shooter_Power);
                sleep(500);
            }
            if (gamepad1.dpad_down) {
                shooter_Power += 0.05;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(-shooter_Power);
                sleep(500);
            }

            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", shooterRight.getPower(), shooterLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", shooter_Power);
            telemetry.update();
        }
    }
}