package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="Pure Pursuit Demo 2", group="Autonomous")

 public class PurePursuitAutoDemo2 extends BasicAuto {
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

		robotUG.driveTrain.robotFieldLocation.setLocation(-24,-63,0);

		telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
		telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y, robotUG.driveTrain.robotFieldLocation.theta);
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

		// for tests and smaller field trials the robot is initialized to (0,0) and 0.0 degrees
		robotUG.driveTrain.robotX = 0;
		robotUG.driveTrain.robotY = 0;

		ArrayList<PursuitPoint> pathPoints = new ArrayList<>();
		pathPoints= fieldPoints;
//
		pathPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

                pathPoints.add(new PursuitPoint(-24,-24));
                pathPoints.add(new PursuitPoint(-44,-24));
                pathPoints.add(new PursuitPoint(-44,-12));
                pathPoints.add(new PursuitPoint(-60,-12));
                pathPoints.add(new PursuitPoint(-60, 1));


		for(int h=0;h<pathPoints.size()-1;h++) {
			lines.add(new PursuitLines(pathPoints.get(h).x, pathPoints.get(h).y, pathPoints.get(h+1).x, pathPoints.get(h+1).y));
		}

		for(int h=0;h<pathPoints.size();h++) {
			telemetry.addData("Point", "%d: %.2f, %.2f", h, pathPoints.get(h).x, pathPoints.get(h).y);
		}

		pressAToContinue();

		// DON'T LOOP -- run through path 1 time
		robotUG.driveTrain.drivePursuit(pathPoints, this, "Drive multi-lines");


		telemetry.addData("Driving Completed", "...successfully?!?");

		telemetry.addLine("----------------------------------");

		telemetry.addData("Robot Heading", " Desired: %.2f, FieldNav: %.2f, RobotHeading: %.2f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotFieldLocation.theta, robotUG.driveTrain.robotHeading);
		telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
				robotUG.driveTrain.targetPoint.x,robotUG.driveTrain.targetPoint.y, robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);

		telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
				robotUG.driveTrain.flPrevious, robotUG.driveTrain.frPrevious, robotUG.driveTrain.brPrevious, robotUG.driveTrain.blPrevious);

		telemetry.addData("Final Point", " (%.2f, %.2f)", pathPoints.get(pathPoints.size()-1).x,pathPoints.get(pathPoints.size()-1).y);
		telemetry.addLine("----------------------------------");
		telemetry.update();
	}

}
