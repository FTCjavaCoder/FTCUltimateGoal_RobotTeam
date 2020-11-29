package UltimateGoal_RobotTeam.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;

@TeleOp(name="Conveyor & Shooter + Drive", group="TeleOp")

public class ConveyorShooterDrive extends BasicTeleOp {

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
         * [5] = ImageRecog
         * items that are 1 = true will be configured to the robot
         */
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
        boolean[] configArray = new boolean[]{ true, 	true, 	true, 		false, 		false,      false};

        robotUG = new HardwareRobotMulti(this, configArray,false);
        // READ HASHMAP FILE
        readOrWriteHashMap();
        // Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
        robotUG.driveTrain.initIMU(this); //confgures IMU and sets initial heading to 0.0 degrees

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

            telemetry.addData("Status", "Run Time: ",runtime.toString());
//			multiTelemetry(telemetryOption);
            telemetry.addData("Robot Heading", "( %.2f )", robotUG.driveTrain.robotHeading);
            telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
                    forwardDirection, rightDirection, clockwise);
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
                    robotUG.driveTrain.backRight.getPower());
            telemetry.addData("Motor Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.shooter.shooterRight.getPower(), robotUG.shooter.shooterLeft.getPower());
            telemetry.addData("Shooter Power Variable", "Variable: Shooter Power (%.2f)", robotUG.shooter.shooter_Power);
            telemetry.addData("Servo Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.conveyor.conveyorRight.getPower(), robotUG.conveyor.conveyorLeft.getPower());
            telemetry.addData("Conveyor Power Variable", "Variable: Shooter Power (%.2f)", robotUG.conveyor.conveyor_Power);
            telemetry.update();

            idle();
        }

        robotUG.driveTrain.setMotorPower(0);

    }

}