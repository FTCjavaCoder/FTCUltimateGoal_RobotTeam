package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import UltimateGoal_RobotTeam.HarwareConfig.Conveyor;
import UltimateGoal_RobotTeam.HarwareConfig.DriveTrain;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

@Autonomous(name="Double Wobble Goal Autonomous Left", group="Autonomous")
@Disabled
 public class BlueEx2WobbleAuto extends BasicAuto {
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
		boolean[] configArray = new boolean[]{ true, 	true, 	true, 		true, 		true,		true};

		// READ HASHMAP FILE
		readOrWriteHashMap();

		robotUG = new HardwareRobotMulti(this, configArray,testModeActive);

		// Tell the robot where it's starting location and orientation on the field is

		robotUG.driveTrain.robotFieldLocation.setLocation(-59,-63,90);
		robotUG.driveTrain.initIMUtoAngle(-robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
		robotUG.driveTrain.robotX = 0;// robot local coordinates always start at 0
		robotUG.driveTrain.robotY = 0;
		robotUG.wobbleArm.wobbleGoalServo.setPosition(0.8);//this is a loose grip

		// Update telemetry to tell driver than robot is ready
		telemetry.addData("STATUS", "MultiRobot Hardware Configured!!");
		for(int j=0;j<configArray.length;j++) {
			telemetry.addData("ConfigArray Index", "%d with Value: %s", j, configArray[j]);
		}
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

		robotUG.wobbleArm.wobbleGoalServo.setPosition(0.9);//this is a firm grip on the goal

		// Add points for Pure Pursuit motion - always start with where the robot was initialized to be on the field

		/* Drive to Wobble Goal and Scan the Number of Rings*/

		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

		fieldPoints.add(new PursuitPoint(-36, -43));

	// Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
		for(int h=0;h<fieldPoints.size();h++) {
		telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To View the Rings");
		/* -- COACH NOTE: overall good progress but several items need to be tested
		 *   - Should create variables either in OpMode or in HW for specific conditions so a parameter can be used
		 *       shooter power, conveyor ON time, right motion, arm & servo positions, and other hard coded values could be parameters
		 *   - varying times for the image recognition (2s works for both waiting and scanning but can be less)
		 *   - path around the rings so the robot won't drive over them
		 *   - updated location to drop the wobble goal inside the drop zone - too far towards the goals currently
		 *   - updated location to shoot power shot targets further behind line
		 *   - need to have collector running because last ring will be under it
		 *   - SUGGESTION try shooting the high goal all from the same spot as an initial autonomous
		 *       reduces complexity of multiple movements
		 *       allows the shooter, conveyor, collector to be on for 1 set time
		 *   - if trying the power shots the robot needs to move more to the right and re-align
		 */

		/* -- COACH NOTE: for IMAGE RECOGNITION developed timed method and ran several times counting loops
		 *   - 2.0s yields 2000 to 9000 loops in timed method
		 *   - 2.0s wait before looking at rings is sufficient time for image to stabilize
		 *   %% suggest checking reduced wait times and reduce number of loops for the looping method %%
		 */
		double start = runtime.time();
		while ((runtime.time() - start) < 2.0){
			// Do nothing but report TM for counter and wait for robot to settle before looking at rings
			robotUG.imageRecog.getTelemetry(this);
			telemetry.update();
		}
//		String ringsViewed = robotUG.imageRecog.viewRings(this, 25);//baseline method that runs for set number of loops
		String ringsViewed = robotUG.imageRecog.viewRingsTimed(this, 0.5);// ALTERNATE method that runs based on time

		/* COACH NOTE: imageRecog methods end with telemetry being added but waiting for a telemetry.update()
		 * -- expect an update in the main OpMode or a pressAToContinue method to follow
		 */
		telemetry.addLine("------------------------------------");
		telemetry.addData("Image Recognition Completed", "String Value: %s", ringsViewed);
		pressAToContinue();


		robotUG.imageRecog.shutdown();//shutdown after pressA to allow the driver to observe screen before moving on

	/* Choose Where to go Next and Pick up Wobble Goal */
		decideWobbleGoalZone(ringsViewed);
		/* -- COACH NOTE: for decideWobbleGoalZone
		 *   - current position at the edge of the wobble goal zone and method to loosely drop goal
		 * 			places the goal too far and outside of zone
		 *   %% suggest moving the robot closer to start
		 *   %% suggest having all robot paths end with straight section ~ 9 inches long that orient robot
		 */
//		for(int h=0;h<fieldPoints.size();h++) {
//		telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
//		}

//		pressAToContinue();// review the Pursuit Points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Wobble Goal drop zone");

		/* COACH ADDITIONS: added helpful telemetry
		 *  - Add telemetry before every "pressAToContinue" to provide updates on the robot's progress
		 *  - Report the step complete, robot position, arm position, etc.
		 */

//		telemetry.addLine("Drive to Wobble Goal Drop Zone Completed");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotHeading);
//		pressAToContinue();// Review robot's motion

	/* Rotate 180*, Drop the Wobble Goal and Rotation 180 */
		robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180*", this);

//		telemetry.addLine("Rotate to Drop Goal");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);
//
//		pressAToContinue();// Review rotation

		robotUG.wobbleArm.dropWobble(this);

//		pressAToContinue();// Review wobble goal drop

		fieldPoints.clear();// clear all the prior points
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));

		// back for second Wobble Goal
		fieldPoints.add(new PursuitPoint(-48, -8));//??
		fieldPoints.add(new PursuitPoint(-30, -8));//??
		pressAToContinue();// Review new pursuit points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Second Wobble Goal");

		robotUG.wobbleArm.grabWobble(this);// Grab second Wobble Goal

		// Go to drop point for second Wobble Goal
		fieldPoints.add(new PursuitPoint(-48, -8));//??
		fieldPoints.add(new PursuitPoint(-30, -8));//??
		pressAToContinue();// Review new pursuit points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To second Wobble Goal drop zone");

		robotUG.wobbleArm.dropWobble(this);// drop second Wobble Goal

		fieldPoints.clear();// clear all the prior points
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y));
		//Add the current robot location so a pursuit path can be found
		//Add the desired points
		/* COACH SUGGESTION: move the robot further behind the shooting line so that any R/L motion will not cross the line
		 *   - moved location from 0 to -6 in Y
		 *   - Added a point at (-24, -6) so robot would align straight from any location
		 */
		fieldPoints.add(new PursuitPoint(-48, -8));/* COACH CHANGED - for high goal - allow all options to align */
		fieldPoints.add(new PursuitPoint(-30, -8));/* COACH CHANGED - for high goal */

		//TURN ON SHOOTER -- allow time to power up to full speed while driving
		robotUG.shooter.setShooter_Power(1.0);//1.0 for high goal too much @ Y = -6, trying -8

	/* Drive to and Shoot the Powershots */
		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");
//		telemetry.addLine("Drive to Shooting Position");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 0.0, robotUG.driveTrain.robotHeading);
//		pressAToContinue();
	/* Coach Note: need to rotate to face the PowerShot targets or HIGh GOAL and activate shooter, conveyor, & collector
	 * see added lines below
	 */


	robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);/* COACH ADDED */

	// make sure you are at -90 angle
//		telemetry.addLine(" VERIFY robot is aligned Shoot Target #1");
//		telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
//		telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
//		pressAToContinue();

		// shoot HIGH GOAL
		//TURN ON CONVEYOR & COLLECTOR (last ring is partially under collector)
		robotUG.conveyor.setMotion(Conveyor.motionType.UP);
		robotUG.collector.collectorWheel.setPower(-1.0);//need negative power to collector rings
		double startTime = runtime.time();
		double shootTime = runtime.time() - startTime;
		while(shootTime <10.0) {//Since no sensors purely timed set of shots
			shootTime = runtime.time() - startTime;
			telemetry.addLine("Shoot high goal x 3");
			telemetry.addData("Timer", " %1.2f", shootTime);
			telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.getShooter_Power());
			telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
			telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
			telemetry.update();
		}
		//TURN OFF CONVEYOR & COLLECTOR OFF
		robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
		robotUG.collector.collectorWheel.setPower(0.0);
		robotUG.shooter.shutdown();
//		telemetry.addData("Time to Shoot Target 3 targets", " %1.2f", shootTime);
//		pressAToContinue();//record the time to fire shot #1 and observe outcome

		/* COACH NOTE: observation that robot not moving far enough right
		 *  7.5" is the correct spacing but the robot rotates so may not fully move the desired amount
		 *   Could update motion using the navigator
		 *   short term increase the motion by 1.0 to 8.5" and try out
		 */
//		robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 8.5, -90, "Move Right 7.5 inch to shot", this);
//
//		//Make sure that robot is lined up for 2nd shot
//		telemetry.addLine(" VERIFY robot is aligned Shoot Target #2");
//		telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
//		telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
//		pressAToContinue();
//		// shoot powershot
//		//TURN ON CONVEYOR & COLLECTOR
//		robotUG.conveyor.setMotion(Conveyor.motionType.UP);
//		robotUG.collector.collectorWheel.setPower(-1.0);// may not be needed but just in case ring is stuck
//
//		startTime = runtime.time();
//		shootTime = runtime.time() - startTime;
//		while(!gamepad2.back) {
//			telemetry.addLine("Shoot Target #2");
//			telemetry.addData("Timer", " %1.2f", runtime.time() - startTime);
//			telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.shooter_Power);
//			telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
//			telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
//			telemetry.update();
//			shootTime = runtime.time() - startTime;
//		}
//		//TURN OFF CONVEYOR & COLLECTOR
//		robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
//		robotUG.collector.collectorWheel.setPower(0.0);
//
//		telemetry.addData("Time to Shoot Target #2", " %1.2f", shootTime);
//		pressAToContinue();//record the time to fire shot #1 and observe outcome
//
//		robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.RightLeft, 8.5, -90, "Move Right 7.5 inch to shot", this);
//		//Make sure that robot is lined up for 2nd shot
//		telemetry.addLine(" VERIFY robot is aligned Shoot Target #3");
//		telemetry.addData("Heading", " %1.1f",  robotUG.driveTrain.robotHeading);
//		telemetry.addData("Location", " (%1.1f, %1.1f)",  robotUG.driveTrain.robotFieldLocation.x,robotUG.driveTrain.robotFieldLocation.y);
//		pressAToContinue();
//		//TURN ON CONVEYOR & COLLECTOR
//		robotUG.conveyor.setMotion(Conveyor.motionType.UP);
//		robotUG.collector.collectorWheel.setPower(-1.0);
//
//		startTime = runtime.time();
//		shootTime = runtime.time() - startTime;
//		while(!gamepad2.back) {
//			telemetry.addLine("Shoot Target #3");
//			telemetry.addData("Timer", " %1.2f", runtime.time() - startTime);
//			telemetry.addData("Shooter Power", "  %1.2f",  robotUG.shooter.shooter_Power);
//			telemetry.addData("Conveyor Power", " %1.1f",  robotUG.conveyor.conveyor_Power);
//			telemetry.addLine("Press GamePad2 'BACK' once shooter fires ...");
//			telemetry.update();
//			shootTime = runtime.time() - startTime;
//		}
//		//TURN OFF CONVEYOR & SHOOTER
//		robotUG.conveyor.setMotion(Conveyor.motionType.OFF);
//		robotUG.shooter.setShooter_Power(0.0);
//		telemetry.addData("Time to Shoot Target #3", " %1.2f", shootTime);
//		pressAToContinue();//record the time to fire shot #1 and observe outcome


		robotUG.driveTrain.IMUDriveFwdRight(DriveTrain.moveDirection.FwdBack, 12, -90, "Move Fwd ~6 in. to score points", this);
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

		pressAToContinue();//observe telemetry before shutdown
		robotUG.shutdownAll();
	}

}