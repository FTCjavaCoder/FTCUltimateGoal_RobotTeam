package UltimateGoal_RobotTeam.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

@TeleOp(name="Pure Pursuit Trouble Shooting", group="Test")
@Disabled
public class PurePursuitTS extends BasicTeleOp {

    @Override
    public void runOpMode() {

        initializeTeleOp();

        waitForStart();

        while (opModeIsActive()) {

            robotUG.driveTrain.williamDrivePower(gamepad1, gamepad2, this);

            robotUG.driveTrain.angleUnWrap();

            robotUG.driveTrain.robotNavigator(this);

            robotUG.driveTrain.robotNavigator(this);

            telemetry.addData("Status", "Run Time: ",runtime.toString());
//			multiTelemetry(telemetryOption);
            telemetry.addData("Robot Heading", "( %.2f )", robotUG.driveTrain.robotHeading);
            telemetry.addData("Commands Drive", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)",
                    forwardDirection, rightDirection, clockwise);
            telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
                    robotUG.driveTrain.backRight.getPower());


            telemetry.addData("Field X", "Variable: Field X V1 (%.2f)", robotUG.driveTrain.robotFieldLocation.x);
            telemetry.addData("Field Y", "Variable: Field Y V1 (%.2f)", robotUG.driveTrain.robotFieldLocation.y);
            telemetry.addData("Field Angle ", "Variable: Field Angle V1 (%.2f)", robotUG.driveTrain.robotFieldLocation.theta);

            telemetry.update();

            idle();
        }
    }


    @Override
    public void initializeTeleOp() {
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
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
        boolean[] configArray = new boolean[]{ true, 	false, 	false, 		false, 		false,      false};

        robotUG = new HardwareRobotMulti(this, configArray,false);
        // READ HASHMAP FILE
        readOrWriteHashMap();
        // Tel the robot that it's starting at (0,0) field center and angle is zero - facing EAST - Right
        robotUG.driveTrain.initIMU(this); //confgures IMU and sets initial heading to 0.0 degrees

        robotUG.driveTrain.robotFieldLocation.setLocation(0,0,0);

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(), robotUG.driveTrain.backRight.getPower());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }


}
