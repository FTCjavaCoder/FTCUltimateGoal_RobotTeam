package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Wobble Goal Arm and Grab Test ONLY", group="Test")

public class WobbleGoalArmGrabTest extends BasicTeleOp {

    @Override
    public void runOpMode() {
        runtime.reset();
        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        // configure the robot needed - for this demo only need DriveTrain
        // configArray has True or False values for each subsystem HW element
        //
        /** configArray is arranged as
         * [0] = DriveTrain
         * [1] = Shooter
         * [2] = Conveyor
         * [3] = WobbleArm
         * [4] = Collector
         * items that are 1 = true will be configured to the robot
         */
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector
        boolean[] configArray = new boolean[]{false, 	false, 	 false, 	true, 	    false};

        robotUG = new HardwareRobotMulti(this, configArray,testModeActive);

        telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
        telemetry.addData("Motor Variable", "Goal Arm Target (%d)", robotUG.wobbleArm.wobbleArmTarget);
        telemetry.addData("Motor Position", "Goal Arm Current Pos (%d)", robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
        telemetry.addData("Servo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                robotUG.wobbleArm.wobbleGrabPos, robotUG.wobbleArm.wobbleReleasePos);
        telemetry.addData("Servo Position", "Servo Pos (%.2f)",
                robotUG.wobbleArm.wobbleGoalPos);
        telemetry.addLine(" ");
        telemetry.addData(">", "Press Play to start");
        // READ HASHMAP FILE
        readOrWriteHashMap();
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            robotUG.wobbleArm.setWobbleMotorPower(gamepad2, this);

            robotUG.wobbleArm.changeWobbleMotorVariable(gamepad2, this);

            robotUG.wobbleArm.setWobbleMotorPosition(gamepad2,this);

            robotUG.wobbleArm.setWobbleServoPos(gamepad2,this);

            telemetry.addData("Arm Target", "Goal Arm Target Angle (%d)", robotUG.wobbleArm.wobbleArmTargetAngle);
            telemetry.addData("Arm Angle", "Goal Arm Current Angle (%.2f)",
                    (robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition() / (cons.DEGREES_TO_COUNTS_60_1 * robotUG.wobbleArm.armGearRatio)));
            telemetry.addData("Motor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d)", robotUG.wobbleArm.armPower, robotUG.wobbleArm.wobbleArmTarget);
            telemetry.addData("Motor Position", "Goal Arm Current Pos (%d)", robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
            telemetry.addData("Servo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    robotUG.wobbleArm.wobbleGrabPos, robotUG.wobbleArm.wobbleReleasePos);
            telemetry.addData("Servo Position", "Servo Pos (%.2f)",
                    robotUG.wobbleArm.wobbleGoalPos);
            telemetry.update();
        }

    }

}