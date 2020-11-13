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
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		false};

		robotUG = new HardwareRobotMulti(this, configArray,false);
		// READ HASHMAP FILE
		readOrWriteHashMap();
		// Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
		robotUG.driveTrain.initIMU(this); //confgures IMU and sets initial heading to 0.0 degrees
		robotUG.driveTrain.robotX = 0;
		robotUG.driveTrain.robotY = 0;
		robotUG.driveTrain.robotLocationV1.setLocation(0,0,0);

		//Indicate initialization complete and provide telemetry
		telemetry.addData("Status: ", "Initialized");
		telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
		telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
	}

//	@Override
	public void runCode() {
// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			// Set Drive Motor Power
			robotUG.driveTrain.drivePowerAllLeftStickScaled(gamepad1, gamepad2, this);

			robotUG.driveTrain.angleUnWrap();

			if (gamepad1.dpad_up) {

				telemetryOption += 1;

				if (telemetryOption > 4) {

					telemetryOption = 0;
				}

			}




			telemetry.addData("Status", "Run Time: ",runtime.toString());
//			multiTelemetry(telemetryOption);
			telemetry.addData("Robot Heading", "( %.2f )", robotUG.driveTrain.robotHeading);
			telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
					forwardDirection, rightDirection, clockwise);
			telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
					robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
					robotUG.driveTrain.backRight.getPower());
			telemetry.update();

			idle();
		}
		robotUG.driveTrain.setMotorPower(0);

	}

}
