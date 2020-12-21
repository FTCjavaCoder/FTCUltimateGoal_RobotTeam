package UltimateGoal_RobotTeam.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
//import TestOpModesOffline.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.util.List;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

@Autonomous(name="BasicTeleOp", group="TeleOp")
@Disabled
public class BasicTeleOp extends BasicOpMode {

    /* Coach Note: clear out unused variables form list below
     * -- remove skystone items
     * -- remove items that can reside in the HW classes (i.e. DriveTrain etc)
     */

    // Define the static power levels and limits for motors
    public double servoStonePos;
    public double clockwise =0;
    public double forwardDirection =0;
    public double rightDirection =0;
    public double verticalDirection = 0;
    public double clockwiseDirection =0;
    public double counterclockwiseDirection = 0;
    public double slideDirection =0;
    public double rightTrigger = 0;
    public double leftTrigger = 0;
    public int slideDistance = 0;
    public double desiredExtend = 0;
    public double desiredRotate = 0;
    public double mineralBoxTrgtPos = 0;
    public double slidePwr = 0;
    public double setJackHeightPos = 3;

    public double capstoneServoPosition = 0;

    public double manualGripperIncrement = 0.05;// was 0.01 and 0.005

    public int telemetryOption = 1;

    /* Coach Note: moved runTime to BasicOpMode so it would be available for all methods receiving om
     *
     */
//    public ElapsedTime runtime = new ElapsedTime(); //create a counter for elapsed time
    @Override
    public void runOpMode() {

    }

    public void initializeTeleOp() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        telemetry.addData(">", "Press Play to start");

        readOrWriteHashMap();

        // initialize configuration to Billy
        Billy.init(hardwareMap, testModeActive);
        // define variables for OpMode powers and positions
        // Initialize all powers and variables to zero

        //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
        Billy.frontLeft.setPower(0);
        Billy.frontRight.setPower(0);
        Billy.backLeft.setPower(0);
        Billy.backRight.setPower(0);
        Billy.shooterLeft.setPower(0);
        Billy.shooterRight.setPower(0);

        Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Billy.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Reset all motor encoders
        Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set all motors to position mode (assumes that all motors have encoders on them)
        Billy.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Billy.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Billy.conveyorLeft.setPower(0);
        Billy.conveyorRight.setPower(0);

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

    }

    public void initializeTeleOpMiniBot() {

        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();

        telemetry.addData(">", "Press Play to start");

        readOrWriteHashMap();

        // initialize configuration to Billy
        Billy.initMiniBot(hardwareMap, testModeActive);
        // define variables for OpMode powers and positions
        // Initialize all powers and variables to zero

        Billy.frontLeft.setPower(0);
        Billy.frontRight.setPower(0);
        Billy.backLeft.setPower(0);
        Billy.backRight.setPower(0);

        Billy.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Billy.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Billy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Billy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset all motor encoders
        Billy.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Billy.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Billy.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Commands", "Forward (%.2f), Right (%.2f), Clockwise (%.2f)", forwardDirection, rightDirection, clockwise);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", Billy.frontLeft.getPower(), Billy.frontRight.getPower(), Billy.backLeft.getPower(), Billy.backRight.getPower());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

    }

    public void stopMotors() {

        Billy.setMotorPower(0);
    }

    public void initServosAfterStart() {

    }

    public void multiTelemetry() {

        if(gamepad1.back){
            setTelemetryOption();
            sleep(300);
        }
        switch (telemetryOption) {

            case 1 :
                telemetry.addLine("DRIVE TRAIN ...");
                if(robotUG.driveTrain != null) {
                    telemetry.addLine("ROBOT GAMEPAD COMMANDS ...");
                    telemetry.addData("\tCommands", "FWD (%.2f), Right (%.2f), CW (%.2f)",
                            robotUG.driveTrain.forwardDirection, robotUG.driveTrain.rightDirection, robotUG.driveTrain.clockwise);
                    telemetry.addData("\tDrive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                            robotUG.driveTrain.frontLeft.getPower(), robotUG.driveTrain.frontRight.getPower(), robotUG.driveTrain.backLeft.getPower(),
                            robotUG.driveTrain.backRight.getPower());
                    telemetry.addLine("ROBOT LOCATION ...");
                    telemetry.addData("\tRobot Field Position (X, Y)", " ( %1.1f, %1.1f)", robotUG.driveTrain.robotFieldLocation.x, robotUG.driveTrain.robotFieldLocation.y);
                    telemetry.addData("\tRobot Angles", " \t Heading: %1.1f, \t Field: %1.1f", robotUG.driveTrain.robotHeading, robotUG.driveTrain.robotFieldLocation.theta);
                }
                else {
                    telemetry.addLine("\t ... is NOT active!!");
                }
                break;

            case 2 :
                telemetry.addLine("WOBBLE GOAL ...");
                if(robotUG.wobbleArm != null) {
                    telemetry.addData("\tArm Target", "Goal Arm Target Angle (%d) degrees", robotUG.wobbleArm.wobbleArmTargetAngle);
                    telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",robotUG.wobbleArm.getArmAngleDegrees());
                    telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", robotUG.wobbleArm.armPower, robotUG.wobbleArm.wobbleGoalArm.getTargetPosition());
                    telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
                    telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                            robotUG.wobbleArm.wobbleGrabPos, robotUG.wobbleArm.wobbleReleasePos);
                    telemetry.addData("\tServo Position", "Servo Pos (%.2f)",robotUG.wobbleArm.wobbleGoalServo.getPosition());
                }
                else {
                    telemetry.addLine("\t ... is NOT active!!");
                }
                break;

            case 3 :
                telemetry.addLine("SHOOTER ...");
                if(robotUG.shooter != null) {
                    telemetry.addData("\tMotor Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.shooter.shooterRight.getPower(), robotUG.shooter.shooterLeft.getPower());
                    telemetry.addData("\tShooter Power Variable", "Variable: Shooter Power (%.2f)", robotUG.shooter.shooter_Power);
                }
                else {
                    telemetry.addLine("\t ... is NOT active!!");
                }
                break;

            case 4 :
                telemetry.addLine("CONVEYOR ...");
                if(robotUG.conveyor != null) {
                    telemetry.addData("\tServo Power", "Right Power (%.2f), Left Power (%.2f)", robotUG.conveyor.conveyorRight.getPower(), robotUG.conveyor.conveyorLeft.getPower());
                    telemetry.addData("\tConveyor Power Variable", "Variable: Conveyor Power (%.2f)", robotUG.conveyor.conveyor_Power);
                }
                else {
                        telemetry.addLine("\t ... is NOT active!!");
                }
                break;
            case 5 :
                telemetry.addLine("COLLECTOR ...");
                if(robotUG.collector != null) {
                    telemetry.addData("\tCommands", "collectorPower (%.2f), collectorWheel Power (%.2f)", robotUG.collector.collectorPower, robotUG.collector.collectorWheel.getPower());
                telemetry.update();
                }
                else {
                    telemetry.addLine("\t ... is NOT active!!");
                }

                break;
            case 6 :
                telemetry.addLine("IMAGE RECOGNITION ...");
                if(robotUG.imageRecog != null){
                    List<Recognition> ringRecognitions = robotUG.imageRecog.tfod.getRecognitions();

                    for (int i = 0; i < ringRecognitions.size(); i++) {
                        telemetry.addData(String.format("\tLabel (%d)", i), ringRecognitions.get(i).getLabel());
                        telemetry.addData(String.format("\t\tleft,top (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getLeft(), ringRecognitions.get(i).getTop());
                        telemetry.addData(String.format("\t\tright,bottom (%d)", i), "%.03f , %.03f",
                                ringRecognitions.get(i).getRight(), ringRecognitions.get(i).getBottom());
                    }
                }
                else {
                    telemetry.addLine("\t ... is NOT active!!");
                }

                break;
        }
        telemetry.update();
    }
    public void setTelemetryOption() {
        if(telemetryOption == 6) {
            telemetryOption = 1;
        }
        else {
            telemetryOption +=1;
        }

    }
}