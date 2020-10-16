package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Conveyor Test", group="Test")

public class ConveyorTest extends BasicTeleOp {

    public CRServo conveyorLeft = null;
    public CRServo conveyorRight = null;

    public double conveyor_Power = 0;

    @Override
    public void runOpMode() {

        conveyorLeft = hardwareMap.get(CRServo.class, "motor_conveyorL");
        conveyorRight = hardwareMap.get(CRServo.class, "motor_conveyorR");

        conveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorLeft.setPower(0);
        conveyorRight.setPower(0);

        // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                conveyor_Power = 1;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(-conveyor_Power);
                sleep(500);
            }
            if (gamepad1.y) {
                conveyor_Power = -1;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(-conveyor_Power);
                sleep(500);
            }
            if (gamepad1.b) {
                conveyor_Power = 0;
                conveyorLeft.setPower(conveyor_Power);
                conveyorRight.setPower(conveyor_Power);
                sleep(500);
            }

            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", conveyorRight.getPower(), conveyorLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", conveyor_Power);
            telemetry.update();
        }
    }
}