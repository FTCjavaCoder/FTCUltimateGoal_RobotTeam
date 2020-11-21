package UltimateGoal_RobotTeam.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;

@TeleOp(name="Everything Drive", group="TeleOp")

public class EverythingDrive extends BasicTeleOp {

    @Override
    public void runOpMode() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        initializeTeleOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runCode();

    } // main opMode end methods go below

    @Override
    public void initializeTeleOp() {
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
        boolean[] configArray = new boolean[]{ true, 	true, 	true, 		true, 		true};

        robotUG = new HardwareRobotMulti(this, configArray,false);
        // READ HASHMAP FILE
        readOrWriteHashMap();
        // Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
        robotUG.driveTrain.initIMU(this); //configures IMU and sets initial heading to 0.0 degrees

        robotUG.driveTrain.robotFieldLocation.setLocation(0,0,0);

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

//    @Override
    public void runCode() {

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Set Drive Motor Power
            robotUG.driveTrain.drivePowerAllLeftStickScaled(gamepad1, gamepad2, this);

            robotUG.driveTrain.angleUnWrap();

            robotUG.conveyor.ConveyorControl(gamepad1, this);

            robotUG.shooter.ShooterControl(gamepad1, this);

            robotUG.wobbleArm.setWobbleMotorPower(gamepad2, this);

            robotUG.wobbleArm.changeWobbleMotorVariable(gamepad2, this);

            robotUG.wobbleArm.setWobbleMotorPosition(gamepad2,this);

            robotUG.wobbleArm.setWobbleServoPos(gamepad2,this);

            robotUG.collector.collectorControl(gamepad2,  this);

            telemetry.addData("Status", "Run Time: ",runtime.toString());
//			multiTelemetry(telemetryOption);
            telemetry.addData("Robot Heading", "( %.2f )", robotUG.driveTrain.robotHeading);
            telemetry.addData("Arm Target", "Goal Arm Target Angle (%d)", robotUG.wobbleArm.wobbleArmTargetAngle);
            telemetry.addData("Arm Angle", "Goal Arm Current Angle (%.2f)",
                    (robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition() / (cons.DEGREES_TO_COUNTS_60_1 * robotUG.wobbleArm.armGearRatio)));
            telemetry.addData("Motor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d)", robotUG.wobbleArm.armPower, robotUG.wobbleArm.wobbleArmTarget);
            telemetry.addData("Motor Position", "Goal Arm Current Pos (%d)", robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
            telemetry.addData("Servo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    robotUG.wobbleArm.wobbleGrabPos, robotUG.wobbleArm.wobbleReleasePos);
            telemetry.addData("Servo Position", "Servo Pos (%.2f)",
                    robotUG.wobbleArm.wobbleGoalPos);

            telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
                    forwardDirection, rightDirection, clockwise);
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
                    robotUG.driveTrain.backRight.getPower());
            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.shooter.shooterRight.getPower(), robotUG.shooter.shooterLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", robotUG.shooter.shooter_Power);
            telemetry.addData("Servo Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.conveyor.conveyorRight.getPower(), robotUG.conveyor.conveyorLeft.getPower());
            telemetry.addData("Conveyor Power Variable", "Variable: Shooter Power (%.2f)", robotUG.conveyor.conveyor_Power);
            telemetry.addData("Commands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());
            telemetry.update();

            idle();
        }

        robotUG.driveTrain.setMotorPower(0.0);
        robotUG.conveyor.conveyorLeft.setPower(0.0);
        robotUG.conveyor.conveyorRight.setPower(0.0);
        robotUG.wobbleArm.wobbleGoalArm.setPower(0.0);
        robotUG.collector.collectorWheel.setPower(0.0);

    }

}