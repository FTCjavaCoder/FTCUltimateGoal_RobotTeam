package UltimateGoal_RobotTeam.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.prefs.BackingStoreException;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobot;

@Autonomous(name="MiniBot Auto", group="Autonomous")

public class MiniBotAuto extends BasicAuto {

    @Override
    public void runOpMode() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        initializeMiniBot();

        waitForStart();

        runtime.reset();

        Billy.initIMU(this);


        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.FwdBack,48,0, "fwd 48 in", this);

        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.RightLeft,-24, 0, "left 24 in", this);

        Billy.IMUDriveRotate(-90, "rotate 90* counter clockwise", this);

        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.FwdBack,-48, -90, "back 48 in", this);

        Billy.IMUDriveRotate(90, "rotate +180* clockwise", this);

        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.RightLeft,44, 90, "right 44 in", this);

        Billy.IMUDriveRotate(0, "rotate 90* counter clockwise", this);

        Billy.IMUDriveFwdRight(HardwareRobot.moveDirection.RightLeft,-24, 0, "left 24 in", this);


    }
}
