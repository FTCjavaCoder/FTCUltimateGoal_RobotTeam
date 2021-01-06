package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.HarwareConfig.Shooter.shooterSide;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Encoder Shooter Test", group="Test")

public class EncoderShooterTest extends BasicTeleOp {


    @Override
    public void runOpMode() {

        initializeTeleOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runCode();
    }
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
        boolean[] configArray = new boolean[]{ false, 	true, 	true, 		false, 		true,       false};
        // READ HASHMAP FILE - do this before robot configuration so parameters are loaded and can be used in robotUG constructors
        readOrWriteHashMap();
        //
        robotUG = new HardwareRobotMulti(this, configArray,false);

        //Indicate initialization complete and provide telemetry
        telemetry.addLine(" ");// blank line
        telemetry.addData("Status: ", "Robot & OpMode Initialized");
        telemetry.addLine(" ");// blank line

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display

    }
    public void runCode() {
        // Create variables and arrays as needed - below obsolete with TM in HW classes
//        double dT = 0.0;
        double[] gainData =robotUG.shooter.getGainArray();
//        double[] speedData;
//        double[] powerDataL;
//        double[] powerDataR;
//        double[] posDataL;
//        double[] posDataR;
        // CONFIGURE THE TELEMETRY DESIRED w/o gamepad
        robotUG.setTelemetrySize(3);//create windows for shooter, conveyor, collector
        robotUG.setTelemetryIndex(0);
        //Options are 0 = inactive, 1 = drivetrain, 2 = WGA, 3 = shooter, 4 = conveyor , 5 = collector, 6 = image recognition
        robotUG.setTelemetryOption(3);//Shooter
        robotUG.setTelemetryIndex(1);
        robotUG.setTelemetryOption(4);//Conveyor
        robotUG.setTelemetryIndex(2);
        robotUG.setTelemetryOption(5);//Collector
        RobotLog.vv("SHOOTER SPEED LOG", "********* NEW LOG ***********");
        RobotLog.vv("Shooter Speed Gains", "\tFF:%.6f\tKP:%.6f\tKI:%.6f",gainData[0],gainData[1],gainData[2]);
        RobotLog.vv("Shooter Speed Headers", "\tTime(s)\tDelta Time (s)\tSpeedActive\tTarget Speed (RPM)\tLeft Speed (RPM)\tRight Speed (RPM)");
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robotUG.conveyor.ConveyorControl(gamepad1, this);// Y, B, X
            robotUG.collector.collectorControl(gamepad2,  this);// triggers
            robotUG.collector.setServoPos(gamepad2,  this);// leftstick Y

            robotUG.shooter.setShooterMode(gamepad2,this);//SET the speedActive boolean to determine motor control mode - dPad Right
                //toggles between GamePad1 power control and GamePad2 external speed control
                //Should set speedActive before 1st commands to avoid integrator wind-up
            robotUG.shooter.setShooterPowerControl(gamepad1,this);//use the GamePad1 to set the shooter powers - dPad
            robotUG.shooter.setShooterSpeed(gamepad2,this);//Updated the internal shooter userTargetRPM - dPad
            //REMOVE gain updates during run and make updates with hashmaps between runs
//            robotUG.shooter.setShooterKPGain(gamepad2,this);//Update the proportional gain for troubleshooting - stick buttons
//            robotUG.shooter.setShooterKIGain(gamepad2,this);//Update the integral gain for troubleshooting - bumpers

            robotUG.shooter.shooterSpeedControl(robotUG.shooter.getUserTargetSpeed(),this);//calculate powers and if speedActive control motor powers

            // ADD logging of data in output to phone log - logging should be included in Shooter.getMaxTelemetry()
            //HardwareRobotMulti.telemetryDetails.MAX or HardwareRobotMulti.telemetryDetails.BASIC set the level of TM output
            // since gamepad Method is called below, the GamePad can be used to override the initial telemetry setting
            robotUG.gamePadMultiTelemetry(this, HardwareRobotMulti.telemetryDetails.MAX);
//
//            telemetry.addData("SHOOTER SPEED MODE", " %s", robotUG.shooter.getSpeedMode());
//            gainData = robotUG.shooter.getGainArray();
//            telemetry.addData("\tKP GAIN", "  %.6f", gainData[1]);
//            telemetry.addData("\tKI GAIN", "  %.7f", gainData[2]);
//            telemetry.addData("\tFF GAIN", "     %.6f", gainData[0]);
//            telemetry.addData("\tdt", "  %.6f seconds", robotUG.shooter.getDT());//check deltaTime and see if sower with more method calls
//            //consider moving telemetry into Shooter class and eliminate function calls
//            telemetry.addLine("-----------------------------------------------------------------------\n");
//
//            posDataL = robotUG.shooter.getPosArray(shooterSide.LEFT);
//            posDataR = robotUG.shooter.getPosArray(shooterSide.RIGHT);
//
//            telemetry.addLine("POSITION:");
//            telemetry.addData("\tCurrent Cnt","L: %.0f, R: %.0f", posDataL[0], posDataR[0]);//returns doubles
//            telemetry.addData("\tPrev Counts","L: %.0f, R: %.0f", posDataL[1], posDataR[0]);
//
//            speedData = robotUG.shooter.getSpeedArray();
//            telemetry.addLine("SPEED:");
//            telemetry.addData("\tTarget(RPM)", "          %.1f,", speedData[3]);
//            telemetry.addData("\tCalculated(RPM)", "Avg:  %.1f, L = %.1f, R = %.1f", speedData[2],speedData[0],speedData[1]);
//
//            powerDataL = robotUG.shooter.getPowerArray(shooterSide.LEFT);
//            powerDataR = robotUG.shooter.getPowerArray(shooterSide.RIGHT);
//            telemetry.addLine("POWER:");
//            telemetry.addData("\tFeed Forward", "%.3f", powerDataL[0]);
//            telemetry.addData("\tFeedback P", "L: %.3f, R: %.3f", powerDataL[1], powerDataR[1]);
//            telemetry.addData("\tFeedback I", "L: %.3f, R: %.3f", powerDataL[2], powerDataR[2]);
//            telemetry.addData("\tTotal", "    L: %.3f, R: %.3f", powerDataL[3], powerDataR[3]);
//            telemetry.addData("\tActual", "   L: %.2f, R: %.2f", powerDataL[4], powerDataR[4]);

//            telemetry.update();
            idle();
        }
        RobotLog.vv("END OF LOG", "%%%%%%%%%%% END LOG %%%%%%%%%%%%%%%");

        robotUG.shutdownAll();
    }
}