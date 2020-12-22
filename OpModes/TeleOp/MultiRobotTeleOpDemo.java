package UltimateGoal_RobotTeam.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;

@TeleOp(name="Robot Multi Demo", group="TeleOp")

 public class MultiRobotTeleOpDemo extends BasicTeleOp {
	@Override
	public void runOpMode() {

		initializeTeleOp();

		waitForStart();

		runCode();

	}

	@Override
	public void initializeTeleOp() {
		runtime.reset();
		telemetry.addLine("NOT READY DON'T PRESS PLAY");
		telemetry.update();
		telemetry.setAutoClear(false);//allow all the lines of telemetry to remain during initialization

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
		// HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector	ImageRecog
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		false,		false};

		robotUG = new HardwareRobotMulti(this, configArray,false);
		// READ HASHMAP FILE
		readOrWriteHashMap();
		// Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
		robotUG.driveTrain.initIMU(this); //confgures IMU and sets initial heading to 0.0 degrees
		robotUG.driveTrain.robotLocationV1.setLocation(0,0,0);

		//Coach Note: Need to set GearRatio (40.0:1 for the main robot)
		robotUG.driveTrain.setGearRatio(40.0, this);

		//Indicate initialization complete and provide telemetry
		telemetry.addLine(" ");// blank line
		telemetry.addData("Status: ", "Initialized");
		telemetry.addData("Robot Field Location", "X = %.1f, Y = %.1f, Theta = %.1f",robotUG.driveTrain.robotFieldLocation.x,
				robotUG.driveTrain.robotFieldLocation.y,robotUG.driveTrain.robotFieldLocation.theta);
		telemetry.addData("Drive Train Gear Ratio", " %.1f : 1",robotUG.driveTrain.gearRatio);
		telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
		telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
		telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display

	}

//	@Override
	public void runCode() {
// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			// Set Drive Motor Power
			robotUG.driveTrain.drivePowerAllLeftStickScaled(gamepad1, gamepad2, this);

			robotUG.driveTrain.angleUnWrap();

			multiTelemetry();

			idle();
		}
		robotUG.shutdownAll();

	}

}
