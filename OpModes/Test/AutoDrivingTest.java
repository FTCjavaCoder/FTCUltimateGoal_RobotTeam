package UltimateGoal_RobotTeam.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobot;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;

@Autonomous(name="Auto Driving Test", group="Test")
@Disabled
public class AutoDrivingTest extends BasicAuto {

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = cons.VUFORIA_KEY;
        parameters.cameraDirection   = cons.CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        //all above lines need to be all autonomous OpMode's runOpMode before initialization

//        foundationPosChange = 0;// 0 for moved, 26 for unmoved Foundation.
//        insideOutside = 0;// 0 for Inside, 24 for Outside

        initialize();

        waitForStart();

        runtime.reset();

        Billy.initIMU(this);

        pressAToContinue();
        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.FwdBack, 60, 0, "IMU Forward 60 Inches", this);

        pressAToContinue();
        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.FwdBack, -60, 0, "IMU Backward 60 Inches", this);

        pressAToContinue();
        Billy.IMUDriveRotate(-90,"Rotate 90 deg CCW", this);

        pressAToContinue();
        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.RightLeft, 60, -90, "IMU Right 60 Inches", this);

        pressAToContinue();
        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.RightLeft, -60, -90, "IMU Left 60 Inches", this);

//        pressAToContinue();
//        Billy.IMUDriveRotate(0,"Rotate 90 deg CW", this);

//        Billy.driveGeneralPower(HardwareBilly.moveDirection.FwdBack, 60, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Forward 60 Inches", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.Rotate, 90, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Rotate 90 CW", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.RightLeft, 60, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Right 60 Inches", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.Rotate, 90, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Rotate 90 CW", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.FwdBack, -60, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Backward 60 Inches", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.Rotate, 90, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Rotate 90 CW", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.RightLeft, -60, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Left 60 Inches", this);
//
//        pressAToContinue();
//
//        Billy.driveGeneralPower(HardwareBilly.moveDirection.Rotate, 90, cons.pHM.get("drivePowerLimit").value, cons.pHM.get("drivePowerMinimum").value, "Rotate 90 CW", this);

        telemetry.addLine("OpMode Complete");
        telemetry.update();
        sleep(1500);
    }
}
