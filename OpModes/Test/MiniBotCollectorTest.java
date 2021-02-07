package UltimateGoal_RobotTeam.OpModes.Test;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="MiniBot Collector Test", group="Test")

 public class MiniBotCollectorTest extends BasicTeleOp {
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
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		true,		false};

		robotUG = new HardwareRobotMulti(this, configArray,false);
		// READ HASHMAP FILE
		readOrWriteHashMap();

		//Indicate initialization complete and provide telemetry

		telemetry.addData("Status: ", "Robot & OpMode Initialized");
		telemetry.addData("Robot Field Location", "X = %.1f, Y = %.1f, Theta = %.1f",robotUG.driveTrain.robotFieldLocation.x,
				robotUG.driveTrain.robotFieldLocation.y,robotUG.driveTrain.robotFieldLocation.theta);
		telemetry.addData("Drive Train Gear Ratio", " %.1f : 1",robotUG.driveTrain.gearRatio);
		telemetry.addLine(" ");// blank line

		telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
		telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
				robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
		telemetry.addData("Commands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
		telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display

	}

//	@Override
	public void runCode() {

		// CONFIGURE THE TELEMETRY DESIRED w/o gamepad prior to loop
		robotUG.setTelemetrySize(2);//create windows for driveTrain, collector
		robotUG.setTelemetryIndex(0);//active index from 0 to size-1
		//Options are 0 = inactive, 1 = drivetrain, 2 = WGA, 3 = shooter, 4 = conveyor , 5 = collector, 6 = image recognition
		robotUG.setTelemetryOption(1);//DriveTrain

		robotUG.setTelemetryIndex(1);
		robotUG.setTelemetryOption(5);//Collector
// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			// Set wheel Motor Power
			robotUG.collector.collectorControl(gamepad1,  this);//This is variable control by the triggers

			robotUG.driveTrain.drivePowerAllLeftStickScaled(gamepad1, gamepad2, this);//Note gamepad2 is not used in method

			robotUG.gamePadMultiTelemetry(this, HardwareRobotMulti.telemetryDetails.MAX);// COACH implemented multiTelemetry

			idle();
		}
		robotUG.shutdownAll();

	}

}
