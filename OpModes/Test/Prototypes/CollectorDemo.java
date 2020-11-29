package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Collector Demo", group="Test")

 public class CollectorDemo extends BasicTeleOp {
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
		 * [5] = ImageRecog
		 * items that are 1 = true will be configured to the robot
		 */
		// HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector	ImageRecog
		boolean[] configArray = new boolean[]{ false, 	false, 	false, 		false, 		true,		false};

		robotUG = new HardwareRobotMulti(this, configArray,false);
		// READ HASHMAP FILE
		readOrWriteHashMap();

		//Indicate initialization complete and provide telemetry
		telemetry.addData("Status: ", "Initialized");
		telemetry.addData("Commands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
	}

//	@Override
	public void runCode() {
// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			// Set wheel Motor Power
			robotUG.collector.collectorControl(gamepad2,  this);



			telemetry.addData("Status", "Run Time: ",runtime.toString());
			telemetry.addData("Commands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());

			telemetry.update();

			idle();
		}
		robotUG.collector.collectorWheel.setPower(0.0);

	}

}
