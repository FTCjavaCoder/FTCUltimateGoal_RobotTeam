package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="Pure Pursuit Demo", group="Autonomous")

 public class PurePursuitAutoDemo extends BasicAuto {
	@Override
	public void runOpMode() {

		initialize();

		waitForStart();

		runCode();

	}

	@Override
	public void initialize() {
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

		robotUG = new HardwareRobotMulti(this, configArray,testModeActive);
		// READ HASHMAP FILE
		readOrWriteHashMap();
		// Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
		robotUG.driveTrain.initIMU(this); //confgures IMU and sets initial heading to 0.0 degrees
		robotUG.driveTrain.robotX1 = 0;
		robotUG.driveTrain.robotY1 = 0;
		robotUG.driveTrain.robotLocationV1.setLocation(0,0,0);
		telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
		telemetry.addData("Robot Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",robotUG.driveTrain.robotLocationV1.x, robotUG.driveTrain.robotLocationV1.y, robotUG.driveTrain.robotLocationV1.theta);
		telemetry.addLine(" ");
		telemetry.addLine("*********************************************");
		telemetry.addData("WARNING", "VERIFY THAT DRIVE POWER LIMIT IS LOW FOR INITIAL TESTS");
		telemetry.addLine("*********************************************");
		telemetry.addLine(" ");
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
	}

	@Override
	public void runCode() {

		ArrayList<PursuitPoint> pathPoints = new ArrayList<>();
		pathPoints= fieldPoints;
		// Always start path with where robot is
		pathPoints.add(new PursuitPoint(robotUG.driveTrain.robotX1 ,robotUG.driveTrain.robotY1));
		//Slalom course robot will start moving forward and then turn left
		pathPoints.add(new PursuitPoint(18,0));
		pathPoints.add(new PursuitPoint(18,48));
		pathPoints.add(new PursuitPoint(36,48));
		pathPoints.add(new PursuitPoint(36,0));
		pathPoints.add(new PursuitPoint(54, 0));
		pathPoints.add(new PursuitPoint(54,48));
		pathPoints.add(new PursuitPoint(58,48));
		pathPoints.add(new PursuitPoint(58,0));


		for(int h=0;h<pathPoints.size()-1;h++) {
			lines.add(new PursuitLines(pathPoints.get(h).x, pathPoints.get(h).y, pathPoints.get(h+1).x, pathPoints.get(h+1).y));
		}

		// Set low speed for initial demo - USE THE EDIT HASHMAP
//		cons.DRIVE_POWER_LIMIT = 0.3;
//		cons.STEERING_POWER_LIMIT = cons.DRIVE_POWER_LIMIT  * 1.0;// scale back power limits as necessary
//		cons.STEERING_POWER_GAIN = 0.1;

		// DON'T LOOP -- run through path 1 time
		robotUG.driveTrain.drivePursuit(pathPoints, this, "Drive multi-lines");

	}

}
