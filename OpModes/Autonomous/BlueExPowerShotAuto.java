package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import UltimateGoal_RobotTeam.HarwareConfig.DriveTrain;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;

@Autonomous(name="Blue Exterior Power Shot Autonomous", group="Autonomous")

 public class BlueExPowerShotAuto extends BasicAuto {
	@Override
	public void runOpMode() {



		constructRobot();

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
		robotUG.driveTrain.robotFieldLocation.setLocation(-54,-63,90);//Moved to separate method

	}
	@Override
	public void initialize() {
		// Tell the robot where it's starting location and orientation on the field is

		robotUG.driveTrain.initIMUtoAngle(-robotUG.driveTrain.robotFieldLocation.theta);//ADDED HERE FOR OFFLINE, NEEDS TO BE IN initialize() method in OpMode
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
		haveBlueWobble1 = true;//Robot is gripping wobble goal

		exteriorDriveToRings(-36, -52, -37, -43, 0.9);

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

	/* Rotate 180*, Drop the Wobble Goal and Rotation 180 */
		robotUG.driveTrain.IMUDriveRotate(90, "Rotate 180*", this);
		robotUG.wobbleArm.dropWobble(this);

//		telemetry.addLine("Rotate to Drop Goal");
//		telemetry.addData("Desired Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.targetPoint.x, robotUG.driveTrain.targetPoint.y);
//		telemetry.addData("Robot Position (X, Y)", " \t\t( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//		telemetry.addData("Robot Angles", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);
//
//		pressAToContinue();// Review rotation
//
//		telemetry.addLine("Drop Goal");
//		telemetry.addData("Wobble Goal Arm", " Command: %1.2f, Actual: %d", robotUG.wobbleArm.wobbleArmTargetAngle, robotUG.wobbleArm.wobbleArmTarget);
//		telemetry.addData("Wobble Goal Servo", " \t Desired: %1.1f, \t Actual: %1.1f", 90.0, robotUG.driveTrain.robotHeading);
//		telemetry.addLine("Check that Wobble Goal Has Been Dropped ...");

//		robotUG.driveTrain.IMUDriveRotate(0, "Rotate 90 deg CCW", this);/* COACH CHANGED */

	/* Drives the Robot to the Shooting area. x1 and y1 are the first coordinates; x2 and y2 are the second. */
		driveToShoot(-48,-6, -14.5, -6, 0.825);

//		pressAToContinue();

	/* Shoot the High Goal. */
		shootPowerShot(-1.0, 3);

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
// ---------- END CODE FOR POWER SHOT ------------------------

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
		if(testModeActive){// Can't access gamePad
			telemetry.update();
			int counts = 0;
			while(counts < 10) {//wait a 1.0s at the end
				telemetry.addLine("WAITING....");
				telemetry.addData("Counts", " %d", counts);
				telemetry.update();
				robotUG.driveTrain.robotNavigator(this);
				counts+=1;
			}
			telemetry.addLine("/////////////////////////////////");
			telemetry.addLine("OpMode Complete");
			telemetry.addData("Counts", " %d", counts);
			telemetry.update();
		}
		else {// PressA included so the runtime and final reported position can be observed
			pressAToContinue();//observe telemetry before shutdown, without pressA the display is cleared
		}
		robotUG.shutdownAll();
	}

}
