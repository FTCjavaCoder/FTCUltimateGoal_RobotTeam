package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="Blue Interior Double Wobble Goal Autonomous", group="Autonomous")
@Disabled
 public class BlueIn2WobbleAuto extends BasicAuto {
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
	public void constructRobot() {
		// This method constructs the robot
		// It was separated from initialize for use in OfflineCode where offline steps are needed
		// with constructed robot before initialize
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
		boolean[] configArray = new boolean[]{true, true, true, true, true, true};

		// READ HASHMAP FILE
		readOrWriteHashMap();

		robotUG = new HardwareRobotMulti(this, configArray, testModeActive);
		// Update telemetry to tell driver than robot is ready
		telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
//		for(int j=0;j<configArray.length;j++) {
//			telemetry.addData("ConfigArray Index", "%d with Value: %s", j, configArray[j]);
//		}
		telemetry.update();

		robotUG.driveTrain.robotFieldLocation.setLocation(-31,-63,90); //MOVED HERE FOR OFFLINE CODE

	}

	@Override
	public void initialize() {

		// Tell the robot where it's starting location and orientation on the field is

		robotUG.driveTrain.initIMUtoAngle(-robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
		updateIMU();
		robotUG.driveTrain.robotX = 0;// robot local coordinates always start at 0
		robotUG.driveTrain.robotY = 0;
		robotUG.wobbleArm.wobbleGoalServo.setPosition(0.8);//this is a loose grip

		telemetry.addData("Robot Field Location", "X = %.2f inch, Y = %.2f inch, Theta = %.2f degrees",robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y, robotUG.driveTrain.robotFieldLocation.theta);
		telemetry.addLine(" ");
		telemetry.addLine("*********************************************");
		telemetry.addData("WARNING", "VERIFY THAT DRIVE POWER LIMIT IS LOW FOR INITIAL TESTS");
		telemetry.addLine("*********************************************");
		telemetry.addLine(" ");
		telemetry.addData(">", "Press Play to start");
		telemetry.update();
		telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display
	}

	@Override
	public void runCode() {
		runtime.reset();
		haveBlueWobble2 = true;//Robot is gripping wobble goal

		interiorDriveToRings(-36, -41, 0.9);

//		robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);

		/* Choose Where to go Next and Pick up Wobble Goal */
		decideWobbleGoalZone(decideRingNumber());

		/* Get Points for Drawing Lines in Visualization */
		fieldSimPoints();

//		pressAToContinue();

		/* Move to the Wobble Goal Drop Zone */
		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Wobble Goal drop zone");

//		telemetry.addLine("Drive to Wobble Goal Drop Zone Completed");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotHeading);
//		pressAToContinue();// Review robot's motion

		/* Drop the Wobble Goal */
		robotUG.wobbleArm.dropWobble(this);

//		pressAToContinue();// Review wobble goal drop
		robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180 counter-clockwise", this);

		fieldPoints.clear();// clear all the prior points
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));

		// back for second Wobble Goal
		fieldPoints.add(new PursuitPoint(centralPositionX, centralPositionY));
		fieldPoints.add(new PursuitPoint(secondWobbleGoalPosX, secondWobbleGoalPosY));
//		pressAToContinue();// Review new pursuit points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Second Wobble Goal");

		robotUG.wobbleArm.grabWobble(this);// Grab second Wobble Goal

		robotUG.driveTrain.IMUDriveRotate(-90, "Rotate 180 clockwise", this);

		// Go to drop point for second Wobble Goal
		fieldPoints.add(new PursuitPoint(centralPositionX, centralPositionY));
		fieldPoints.add(new PursuitPoint(secondWobbleGoalX, (secondWobbleGoalY - 10) ));
		fieldPoints.add(new PursuitPoint(secondWobbleGoalX, secondWobbleGoalY));
//		pressAToContinue();// Review new pursuit points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To second Wobble Goal drop zone");

		robotUG.wobbleArm.dropWobble(this);// drop second Wobble Goal

		fieldPoints.clear();// clear all the prior points
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));

		//Add the current robot location so a pursuit path can be found
		//Add the desired points

		//park on line

//		robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack, 12, -90, "Move Fwd ~6 in. to score points", this);
		/* INCREASED DRIVING DISTANCE BASED ON SHOOTING LOCATION*/

		//Telemetry output after driving completed
		telemetry.addData("Driving Completed", "...successfully?!?");

		telemetry.addLine("----------------------------------");
		telemetry.addData("Timer", "%.1f",runtime.time());

		telemetry.addData("Robot Heading", " Desired: %.2f, FieldNav: %.2f, RobotHeading: %.2f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotFieldLocation.theta, robotUG.driveTrain.robotHeading);
		telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
				robotUG.driveTrain.targetPoint.x,robotUG.driveTrain.targetPoint.y, robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);

		telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
				robotUG.driveTrain.flPrevious, robotUG.driveTrain.frPrevious, robotUG.driveTrain.brPrevious, robotUG.driveTrain.blPrevious);

		telemetry.addData("Final Pursuit Point", " (%.2f, %.2f)", fieldPoints.get(fieldPoints.size()-1).x,fieldPoints.get(fieldPoints.size()-1).y);
		telemetry.addLine("----------------------------------");
		telemetry.addLine("Observe telemetry and Press A to shutdown");

		robotUG.shutdownAll();
	}

}
