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

//		/* COACH SUGGESTION: move the robot further behind the shooting line so that any R/L motion will not cross the line
//		 *   - moved location from 0 to -6 in Y
//		 *   - Added a point at (-24, -6) so robot would align straight from any location
//		 */
//		fieldPoints.add(new PursuitPoint(-48, -8));/* COACH CHANGED - for high goal - allow all options to align */
//		fieldPoints.add(new PursuitPoint(-30, -8));/* COACH CHANGED - for high goal */
//
//		//TURN ON SHOOTER -- allow time to power up to full speed while driving
//		robotUG.shooter.setShooter_Power(1.0);//1.0 for high goal too much @ Y = -6, trying -8
//
//	/* Drive to and Shoot the Powershots */
//		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");
////		telemetry.addLine("Drive to Shooting Position");
////		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
////		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
////		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 0.0, robotUG.driveTrain.robotHeading);
////		pressAToContinue();
//	/* Coach Note: need to rotate to face the PowerShot targets or HIGh GOAL and activate shooter, conveyor, & collector
//	 * see added lines below
//	 */
//
//
//	robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);/* COACH ADDED */
//
//	// make sure you are at -90 angle
////		telemetry.addLine(" VERIFY robot is aligned Shoot Target #1");
////		telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
////		telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
////		pressAToContinue();
//
//		// shoot HIGH GOAL
//		//TURN ON CONVEYOR & COLLECTOR (last ring is partially under collector)
//		robotUG.conveyor.setMotion(Conveyor.motionType.UP);
//		robotUG.collector.collectorWheel.setPower(-1.0);//need negative power to collector rings
//		double startTime = runtime.time();
//		double shootTime = runtime.time() - startTime;
//		while(shootTime <10.0) {//Since no sensors purely timed set of shots
//			shootTime = runtime.time() - startTime;
//			telemetry.addLine("Shoot high goal x 3");
//			telemetry.addData("Timer", " %1.2f", shootTime);
//			telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.getShooter_Power());
//			telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
//			telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
//			telemetry.update();
//		}
//		//TURN OFF CONVEYOR & COLLECTOR OFF
//		robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
//		robotUG.collector.collectorWheel.setPower(0.0);
//		robotUG.shooter.shutdown();
////		telemetry.addData("Time to Shoot Target 3 targets", " %1.2f", shootTime);
////		pressAToContinue();//record the time to fire shot #1 and observe outcome
//
//		/* COACH NOTE: observation that robot not moving far enough right
//		 *  7.5" is the correct spacing but the robot rotates so may not fully move the desired amount
//		 *   Could update motion using the navigator
//		 *   short term increase the motion by 1.0 to 8.5" and try out
//		 */

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
