package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import UltimateGoal_RobotTeam.HarwareConfig.DriveTrain;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="Wobble Zone Drive Pure Pursuit", group="Autonomous")

 public class CompleteAutonomous extends BasicAuto {
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
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		true, 		false,		true};/// for MiniBot

		robotUG = new HardwareRobotMulti(this, configArray,testModeActive);
		// READ HASHMAP FILE
		readOrWriteHashMap();
		// Tell the robot where it's starting location and orientation on the field is

		robotUG.driveTrain.robotFieldLocation.setLocation(-36,-63,90);
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

		/** Drive to Wobble Goal and Scan the Number of Rings*/
			fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

			fieldPoints.add(new PursuitPoint(-36, -43));

		// Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
			for(int h=0;h<fieldPoints.size();h++) {
			telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

			robotUG.driveTrain.drivePursuit(fieldPoints,this,"To The Wobble Goal");

			String ringsViewed = robotUG.imageRecog.viewRings(this, 100);
			robotUG.imageRecog.shutdown();

			telemetry.addData("String Value: ", ringsViewed);

		/** Choose Where to go Next and Pick up Wobble Goal */
			decideWobbleGoalZone(ringsViewed);

			for(int h=0;h<fieldPoints.size();h++) {
			telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

			//pick up wobble goal

			pressAToContinue();

			robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Wobble Goal drop zone");

			pressAToContinue();

		/** Rotate 180*, Drop the Wobble Goal and Rotation 180 */
			robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180*", this);

			pressAToContinue();

			robotUG.wobbleArm.dropWobble(this);

			pressAToContinue();

			robotUG.driveTrain.IMUDriveRotate(-90, "Rotate 180*", this);

			fieldPoints.add(new PursuitPoint(-20, 0));

			pressAToContinue();

		/** Drive to and Shoot the Powershots */
			robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");

			// make sure you at -90 angle
			// shoot powershot

			pressAToContinue();

			robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Right 7.5 inch to shot", this);

			//shoot powershot

			pressAToContinue();

			robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 7.5, -90, "Move Right 7.5 inch to shot", this);

			//shoot powershot

			pressAToContinue();

			robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack, 6, -90, "Move Fwd ~6 in. to score points", this);

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
