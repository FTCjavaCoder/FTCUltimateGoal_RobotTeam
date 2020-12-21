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
		boolean[] configArray = new boolean[]{ true, 	false, 	false, 		true, 		false,		true};

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

		// Add points for Pure Pursuit motion - always start with where the robot was initialized to be on the field

		/** Drive to Wobble Goal and Scan the Number of Rings*/
		fieldPoints.add(new PursuitPoint(robotUG.driveTrain.robotFieldLocation.x  ,robotUG.driveTrain.robotFieldLocation.y));

		fieldPoints.add(new PursuitPoint(-36, -43));

	// Display the robot points on the screen to confirm what was entered - needed for troubleshooting only
		for(int h=0;h<fieldPoints.size();h++) {
		telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To View the Rings");
		/* -- COACH NOTE: for IMAGE RECOGNITION developed timed method and ran several times counting loops
		 *   - 2.0s yields 2000 to 9000 loops in timed method
		 *   - 2.0s is not enough time for tensor flow to recognize if the rings have been switch if moved during that time
		 *   - either method works if the image is stable BEFORE looping
		 *   - best approach may be to add a sleep BEFORE starting the method to make sure that robot is stopped and settled
		 *   - see addition below which allows OpMode to Exit and provides info during wait/sleep
		 *   - Add telemetry before every "pressAToContinue" to provide updates on the robot's progress
		 */

		double start = runtime.time();
		while ((runtime.time() - start) < 2.0){
			// Do nothing but report TM for counter and wait for robot to settle before looking at rings
			telemetry.addLine("Waiting for robot to stabilize...");
			telemetry.addData("Timer: ", "%1.3fs",runtime.time() - start);
			telemetry.update();
		}
		String ringsViewed = robotUG.imageRecog.viewRings(this, 100);//baseline method that runs for set number of loops
//		String ringsViewed = robotUG.imageRecog.viewRingsTimed(this, 2.0);// ALTERNATE method that runs based on time

		/* COACH NOTE: imageRecog methods end with telemetry being added but waiting for a telemetry.update()
		 * -- expect an update in the main OpMode or a pressAToContinue method to follow
		 *   -
		 */
		telemetry.addLine("------------------------------------");
		telemetry.addData("Image Recognition Completed", "String Value: %s", ringsViewed);
		pressAToContinue();


		robotUG.imageRecog.shutdown();//shutdown after pressA to allow the driver to observe screen before moving on

	/** Choose Where to go Next and Pick up Wobble Goal */
		decideWobbleGoalZone(ringsViewed);

		for(int h=0;h<fieldPoints.size();h++) {
		telemetry.addData("Point", "%d: %.2f, %.2f", h, fieldPoints.get(h).x, fieldPoints.get(h).y);
		}

		pressAToContinue();// review the Pursuit Points

		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To Wobble Goal drop zone");

		/* COACH ADDITIONS: added helpful temetry
		 *  - Add telemetry before every "pressAToContinue" to provide updates on the robot's progress
		 *  - Report the step complete, robot position, arm position, etc.
		 */

		telemetry.addLine("Drive to Wobble Goal Drop Zone Completed");
		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", robotUG.driveTrain.targetHeading, robotUG.driveTrain.robotHeading);
		pressAToContinue();// Review robot's motion

	/** Rotate 180*, Drop the Wobble Goal and Rotation 180 */
		robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180*", this);

		telemetry.addLine("Rotate to Drop Goal");
		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);

		pressAToContinue();// Review rotation

		robotUG.wobbleArm.dropWobble(this);

		telemetry.addLine("Drop Goal");
		telemetry.addData("Wobble Goal Arm", " Command: %1.2f, Actual: %1.2f", robotUG.wobbleArm.wobbleArmTargetAngle, robotUG.wobbleArm.wobbleArmTarget);
		telemetry.addData("Wobble Goal Servo", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);
		telemetry.addLine("Check that Wobble Goal Has Been Dropped ...");

		pressAToContinue();//Review wobble goal drop

	/** Coach Note: don't need to return to the original position as long as Navigator is called in IMUDriveRotate
	 * see changed lines below to face the next point
	 * Alternatively the prior points can be cleared before the new point is added and robot won't need to rotate
	 * (without clearing points robot will go back the way it came)
	 */
		robotUG.driveTrain.IMUDriveRotate(0, "Rotate 90 deg CCW", this);/* COACH CHANGED */

		fieldPoints.add(new PursuitPoint(-14, 0));/* COACH CHANGED - need to go further - robot radius end condition*/

		pressAToContinue();

	/** Drive to and Shoot the Powershots */
		robotUG.driveTrain.drivePursuit(fieldPoints,this,"To PowerShot Shooting Position");

	/** Coach Note: need to rotate to face the PowerShot targets
	 * see added lines below
	 */


	robotUG.driveTrain.IMUDriveRotate(-90, "Rotate to Face Targets", this);/* COACH ADDED */

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
