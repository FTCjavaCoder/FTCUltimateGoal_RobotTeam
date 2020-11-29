package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Wobble Goal Arm Stick Control", group="Test")

public class WobbleGoalArmStickControl extends BasicTeleOp {

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
         * [5] = ImageRecog
         * items that are 1 = true will be configured to the robot
         */
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
        boolean[] configArray = new boolean[]{false, 	false, 	 false, 	true, 	    false,      false};

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

//            robotUG.wobbleArm.wobbleArmStickControl(gamepad2,this);

            robotUG.wobbleArm.setWobbleServoPos(gamepad2,this);

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