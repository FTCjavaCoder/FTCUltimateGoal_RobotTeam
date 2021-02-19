package UltimateGoal_RobotTeam.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;

@TeleOp(name="Main Drive Speed Control Shoot", group="TeleOp")

public class MainDriveSpeedControlShoot extends BasicTeleOp {

    @Override
    public void runOpMode() {

        initializeTeleOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runCode();

    } // main opMode end methods go below

    @Override
    public void initializeTeleOp() {
        runtime.reset();
        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();
        telemetry.setAutoClear(false);//allow all the lines of telemetry to remain during initialization

        // configure the robot needed
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
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
        boolean[] configArray = new boolean[]{ true, 	true, 	true, 		true, 		true,       false};
// TROUBLESHOOTING: report values of configArray
//        for(int j=0;j<configArray.length;j++) {
//            telemetry.addData("ConfigArray Index", "%d with Value: %s", j, configArray[j]);
//        }
//        telemetry.update();

        robotUG = new HardwareRobotMulti(this, configArray,false);
        // READ HASHMAP FILE
        readOrWriteHashMap();
        // Tell the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
        robotUG.driveTrain.robotFieldLocation.setLocation(0,0,0);

        robotUG.driveTrain.initIMUtoAngle(-robotUG.driveTrain.robotFieldLocation.theta);// set IMU to desired robotHeading

        //Coach Note: Need to set GearRatio
        robotUG.driveTrain.setGearRatio(40.0);


        //Indicate initialization complete and provide telemetry
        telemetry.addLine(" ");// blank line
        telemetry.addData("Status: ", "Robot & OpMode Initialized");
        telemetry.addData("Robot Field Location", "X = %.1f, Y = %.1f, Theta = %.1f",robotUG.driveTrain.robotFieldLocation.x,
                robotUG.driveTrain.robotFieldLocation.y,robotUG.driveTrain.robotFieldLocation.theta);
        telemetry.addData("Drive Train Gear Ratio", " %.1f : 1",robotUG.driveTrain.gearRatio);
        telemetry.addLine(" ");// blank line

        telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display

    }

//    @Override
    public void runCode() {

// run until the end of the match (driver presses STOP)
        // CONFIGURE THE TELEMETRY DESIRED w/o gamepad prior to loop
        robotUG.setTelemetrySize(1);//create windows for driveTrain, wobbleGoal, shooter, conveyor, collector
//        robotUG.setTelemetryIndex(0);//active index from 0 to size-1
        //Options are 0 = inactive, 1 = drivetrain, 2 = WGA, 3 = shooter, 4 = conveyor , 5 = collector, 6 = image recognition
//        robotUG.setTelemetryOption(1);//DriveTrain

//        robotUG.setTelemetryIndex(1);
//        robotUG.setTelemetryOption(2);//Wobble Goal Arm

        robotUG.setTelemetryIndex(0);
        robotUG.setTelemetryOption(3);//Shooter

//        robotUG.setTelemetryIndex(3);
//        robotUG.setTelemetryOption(4);//Conveyor
//
//        robotUG.setTelemetryIndex(4);
//        robotUG.setTelemetryOption(5);//Collector
        double[] gainData =robotUG.shooter.getGainArray();

        RobotLog.ii("SHOOTER SPEED LOG", "********* NEW LOG ***********");
        RobotLog.ii("Shooter Speed Gains", "\tFF:%.6f\tKP:%.6f\tKI:%.6f",gainData[0],gainData[1],gainData[2]);
        RobotLog.ii("Shooter Speed Headers", "\tTime(s)\tDelta Time (s)\tSpeedActive\tTarget Speed (RPM)\tLeft Speed (RPM)\tRight Speed (RPM)\tLeft Power\tRight Power");
        runtime.reset();
        //Configuration can be updated within the robotUG.gamePadMultiTelemetry() method later
        while (opModeIsActive()) {

            // Set Drive Motor Power
            robotUG.driveTrain.drivePowerAllLeftStickScaled(gamepad1, gamepad2, this);

            /* -- COACH SUGGESTION: replace angleUnwrap with robotNavigator
             * -- robotNavigator calls angleUnwrap and allows navigation telemetry to be acesses
             */
//            robotUG.driveTrain.angleUnWrap();
            robotUG.driveTrain.robotNavigator(this);

            robotUG.conveyor.ConveyorControl(gamepad2, this);


            robotUG.shooter.setShooterModeGamePad(gamepad2,this);//SET the speedActive boolean to determine motor control mode - dPad Right

            robotUG.shooter.setShooterSpeed(gamepad2,this);//Updated the internal shooter userTargetRPM - dPad

            robotUG.shooter.shooterSpeedControl(robotUG.shooter.getUserTargetSpeed(),this);//calculate powers and if speedActive control motor powers


            robotUG.wobbleArm.setWobbleMotorPower(gamepad1, this);

            robotUG.wobbleArm.setWobbleGoalArmUp(gamepad1, this);

            robotUG.wobbleArm.setWobbleGoalArmDown(gamepad1,this);

            robotUG.wobbleArm.setWobbleGoalArmOverWall(gamepad1,this);

            robotUG.wobbleArm.setWobbleServoPos(gamepad1,this);

            robotUG.collector.collectorOnOffControl(gamepad2,  this);

            robotUG.collector.setServoPos(gamepad2, this);

            /* --Coach Note: add selectable telemetry so based on what's going on we can set TM output
            * -- Are there buttons left to select TM?
            * -- If not selectable then format to easier to read - spacing, tabs, line separators
            */


			robotUG.gamePadMultiTelemetry(this, HardwareRobotMulti.telemetryDetails.MAX);// COACH implemented multiTelemetry
            /* MAX telemetry option returns all the shooter control info and logs data*/

//            telemetry.addLine("ROBOT GAMEPAD COMMANDS ...");
//            telemetry.addData("\tCommands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
//                    forwardDirection, rightDirection, clockwise);
//            telemetry.addData("\tDrive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
//                    robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
//                    robotUG.driveTrain.backRight.getPower());
//
//            telemetry.addLine("ROBOT LOCATION ...");
//            telemetry.addData("\tRobot Field Position (X, Y)", " ( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
//            telemetry.addData("\tRobot Angles", " \t Heading: %1.1f, \t Field: %1.1f", robotUG.driveTrain.robotHeading, robotUG.driveTrain.robotFieldLocation.theta);
//
//            telemetry.addLine("WOBBLE GOAL ...");
//            telemetry.addData("\tArm Target", "Goal Arm Target Angle (%d) degrees", robotUG.wobbleArm.wobbleArmTargetAngle);
////            telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",
////                    (robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition() / (cons.DEGREES_TO_COUNTS_60_1 * robotUG.wobbleArm.ARM_GEAR_RATIO)));
//            telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",robotUG.wobbleArm.getArmAngleDegrees());
//            // Coach Note: for above line make a method in wobbleArm to return the arm position in degrees to make telemetry at this level easier (done!)
//            telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", robotUG.wobbleArm.armPower, robotUG.wobbleArm.wobbleGoalArm.getTargetPosition());
//            telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
//            telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
//                    robotUG.wobbleArm.wobbleGrabPos, robotUG.wobbleArm.wobbleReleasePos);
//            telemetry.addData("\tServo Position", "Servo Pos (%.2f)",robotUG.wobbleArm.wobbleGoalServo.getPosition());
//
//            telemetry.addLine("SHOOTER ...");
//            telemetry.addData("\tMotor Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.shooter.shooterRight.getPower(), robotUG.shooter.shooterLeft.getPower());
//            telemetry.addData("\tShooter Power Variable", "Variable: Shooter Power (%.2f)", robotUG.shooter.shooter_Power);
//
//            telemetry.addLine("CONVEYOR ...");
//            telemetry.addData("\tServo Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.conveyor.conveyorRight.getPower(), robotUG.conveyor.conveyorLeft.getPower());
//            telemetry.addData("\tConveyor Power Variable", "Variable: Conveyor Power (%.2f)", robotUG.conveyor.conveyor_Power);
//
//            telemetry.addLine("COLLECTOR ...");
//            telemetry.addData("\tCommands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());
//            telemetry.update();

            idle();
        }

        /* -- COACH SUGGESTION: create a shutdown or allStop method in robotUG than contains the stop methods
         * -- method would be required to run through only the existing hardware as configured and call the stop methods for each
         * -- stop methods would shutdown all motors/servos
         * -- does the same thing as below but does it simply in the OpMode and is flexible for any HW config
         */
//        robotUG.driveTrain.setMotorPower(0.0);
//        robotUG.conveyor.conveyorLeft.setPower(0.0);
//        robotUG.conveyor.conveyorRight.setPower(0.0);
//        robotUG.wobbleArm.wobbleGoalArm.setPower(0.0);
//        robotUG.collector.collectorWheel.setPower(0.0);
        robotUG.shutdownAll();

    }

}