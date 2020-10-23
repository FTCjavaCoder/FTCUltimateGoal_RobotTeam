package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Conveyor Shooter Test", group="Test")

public class ConveyorShooterTest extends BasicTeleOp {

    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;

    public CRServo conveyorLeft = null;
    public CRServo conveyorRight = null;

    public double shooter_Power = 0;
    public double conveyor_Power = 0;

    @Override
    public void runOpMode() {

        shooterLeft = hardwareMap.get(DcMotor.class, "motor_shooterL");
        shooterRight = hardwareMap.get(DcMotor.class, "motor_shooterR");

        conveyorLeft = hardwareMap.get(CRServo.class, "servo_conveyorL");
        conveyorRight = hardwareMap.get(CRServo.class, "servo_conveyorR");

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorLeft.setPower(0);
        conveyorRight.setPower(0);

        telemetry.addLine("Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                shooter_Power = 0;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(shooter_Power);
                sleep(300);
            }

            if (gamepad1.dpad_up) {
                shooter_Power -= 0.05;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(-shooter_Power);
                sleep(300);
            }
            if (gamepad1.dpad_down) {
                shooter_Power += 0.05;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(-shooter_Power);
                sleep(300);
            }

            if (gamepad1.x) {
                conveyor_Power = 1;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(-conveyor_Power);
                sleep(300);
            }
            if (gamepad1.y) {
                conveyor_Power = -1;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(-conveyor_Power);
                sleep(300);
            }
            if (gamepad1.b) {
                conveyor_Power = 0;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(conveyor_Power);
                sleep(300);
            }

            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", shooterRight.getPower(), shooterLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", shooter_Power);
            telemetry.addData("Servo Power", "Right Power (%.2f), Left Power (%.2f)", conveyorRight.getPower(), conveyorLeft.getPower());
            telemetry.addData("Conveyor Power Variable", "Variable: Shooter Power (%.2f)", conveyor_Power);
            telemetry.update();
        }
    }
}