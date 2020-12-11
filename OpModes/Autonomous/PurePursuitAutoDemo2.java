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
		//output telemetry at the end of running the loop -- will it stay on the screen?
		telemetry.addData("Driving Completed", "...successfully?!?");

		telemetry.addLine("----------------------------------");

		telemetry.addData("Robot Heading", " Desired: %.2f, FieldNav: %.2f, RobotHeading: %.2f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotFieldLocation.theta, robotUG.driveTrain.robotHeading);
		telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
				robotUG.driveTrain.targetPoint.x,robotUG.driveTrain.targetPoint.y, robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);

		telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
				robotUG.driveTrain.flPrevious, robotUG.driveTrain.frPrevious, robotUG.driveTrain.brPrevious, robotUG.driveTrain.blPrevious);

		telemetry.addData("Final Point", " (%.2f, %.2f)", fieldPoints.get(fieldPoints.size()-1).x,fieldPoints.get(fieldPoints.size()-1).y);
		telemetry.addLine("----------------------------------");
		telemetry.update();
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
		 * [5] = ImageRecog
		 * items that are 1 = true will be configured to the robot
		 */
		// HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector	ImageRecog
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		false,		false};

		robotUG = new HardwareRobotMulti(this, configArray,testModeActive);
		// READ HASHMAP FILE
		readOrWriteHashMap();
		// Tell the robot where it's starting location and orientation on the field is

		robotUG.driveTrain.robotFieldLocation.setLocation(-24,-63,90);
		robotUG.driveTrain.initIMUtoAngle(this,-robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
		robotUG.driveTrain.robotX = 0;// robot local coordinates always start at 0
		robotUG.driveTrain.robotY = 0;

		// Update telemetry to tell driver than robot is ready
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

		// Add points for Pure Pursuit motion - always start with where the robot was initialized to be on the field
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));
		// simple drive forward
//		fieldPoints.add(new PursuitPoint(-24,-60));
////		fieldPoints.add(new PursuitPoint(-24,-55));
////		fieldPoints.add(new PursuitPoint(-24,-50));
////		fieldPoints.add(new PursuitPoint(-24,-45));
		// William's points
		fieldPoints.add(new PursuitPoint(-24,-24));
		fieldPoints.add(new PursuitPoint(-44,-24));
		fieldPoints.add(new PursuitPoint(-44,-12));
		fieldPoints.add(new PursuitPoint(-60,-12));
		fieldPoints.add(new PursuitPoint(-60,1));

		// add lines for Visualization only on offline - comment out for real robot
//		for(int h=0;h<fieldPoints.size()-1;h++) {
//			lines.add(new PursuitLines(fieldPoints.get(h).x, fieldPoints.get(h).y, fieldPoints.get(h+1).x, fieldPoints.get(h+1).y));
//		}

		// Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
		for(int h=0;h<fieldPoints.size();h++) {
			telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

		pressAToContinue(); // troubleshooting only

		// DON'T LOOP -- run through path 1 time
		robotUG.driveTrain.drivePursuit(fieldPoints, this, "Drive multi-lines");

		// Can add other sets of paths and driving operation here..

		//Telemetry output after driving completed
		telemetry.addData("Driving Completed", "...successfully?!?");

		telemetry.addLine("----------------------------------");

		telemetry.addData("Robot Heading", " Desired: %.2f, FieldNav: %.2f, RobotHeading: %.2f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotFieldLocation.theta, robotUG.driveTrain.robotHeading);
		telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
				robotUG.driveTrain.targetPoint.x,robotUG.driveTrain.targetPoint.y, robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);

		telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
				robotUG.driveTrain.flPrevious, robotUG.driveTrain.frPrevious, robotUG.driveTrain.brPrevious, robotUG.driveTrain.blPrevious);

		telemetry.addData("Final Point", " (%.2f, %.2f)", fieldPoints.get(fieldPoints.size()-1).x,fieldPoints.get(fieldPoints.size()-1).y);
		telemetry.addLine("----------------------------------");
		telemetry.update();
	}

}
