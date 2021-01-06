package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

public class WobbleArm {

    /* Public OpMode members. */
    public DcMotor wobbleGoalArm    = null;
    public Servo   wobbleGoalServo  = null;

//    Constants cons = new Constants();
    /** -- COACH NOTE: already have Constants in the OpMode, don't include a new instance here
     * use om.cons... instead see below with corrections
    */

    /*  COACH NOTE: should make many of these values 'private' below
     *  - don't need to be public to other programs
     *  - are likely to be final constant values after a build
     *  - if there are values that we want to change as hashmap parameters then they can be initialized in constructor
     *  - BEST PRACTICE: define the variables below with description and units (degrees, inches, counts, power)
     */
    public double wobbleGoalPos = 0.5;// undecided values
    public double wobbleGrabInc = 0.1;
    public double wobbleGrabPos = 0.5;
    public double wobbleReleasePos = 0;
    public int wobbleArmTarget = 0;//KS: this can be deleted it's redundant when there's a conversion method
    public double wobbleArmTargetAngle = 0.0;// this needs be a double - might want decimal degrees
    public int armDegInc = 1;
    public int armDegIncBig = 25; // KS - increments could be candidates for hashmap parameter if need to carefully set angles
    public double armPower = 0.25;
    public double powerInc = 0.05;
    public double armPowerHold = 0.7;
    public final double ARM_GEAR_RATIO = 24/15; //Coach Note: updated value is 24.0/15.0, but will change the angles for the arm by (24/15 / 2)
            // Example WAS 110 = 110/2 * (24/15) = 88, HAVE NOT CHANGED VALUES
            // Updated for constant style guide
    private final double MOTOR_DEG_TO_COUNT = 1440.0/360.0; // Coach Note: since motor is part of this HW can make local
        // don't need to change with any global parameter and therefore avoids having to pass in om.cons for final value

    public int c = 0; //temporary variable use to prevent us from choosing how we want to progress before we drop the wobble goal

    public WobbleArm(BasicOpMode om, boolean tm)  {
        if(tm) {
//            wobbleGoalServo = new Servo();
            om.telemetry.addData("ERROR: ", "Initializing WobbleGoalArm in TestMode...");

        }
        else {
            om.telemetry.addData("Wobble Arm", " Initializing...");
            om.telemetry.update();
            wobbleGoalArm = om.hardwareMap.get(DcMotor.class, "motor_wobble_goal");
            wobbleGoalServo = om.hardwareMap.get(Servo.class, "wobble_goal_servo");

            wobbleGoalArm.setPower(0);
            wobbleGoalArm.setTargetPosition(0);
            wobbleGoalArm.setDirection(DcMotorSimple.Direction.FORWARD);
            wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();
        }
    }

    public void setWobbleMotorPower(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.left_bumper) {
            armPower -= powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }
        if (gamepad.right_bumper) {
            armPower += powerInc;
            wobbleGoalArm.setPower(armPower);
            om.sleep(300);
        }

//        if (gamepad.b) {
//            armPower = 0.0;
//            wobbleGoalArm.setPower(armPower);
//            om.sleep(300);
//        }

    }

    public void changeWobbleMotorVariable(Gamepad gamepad, BasicOpMode om) {

        /*  COACH SUGGESTION: keep the updating of angles in degrees of the arm (output)
         *  - allows removal of the redundant conversions in each if statement to counts
         *  - when setting the target position invoke a conversion method (see newly created method)
         *  - implemented suggestions below and commented orginal
         */
        if (gamepad.dpad_up) {
            wobbleArmTargetAngle += armDegInc;
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO)); // KS added om.cons & commented
            om.sleep(300);
        }
        if (gamepad.dpad_down) {
            wobbleArmTargetAngle -= armDegInc;
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT* ARM_GEAR_RATIO));// KS added om.cons & commented
            om.sleep(300);
        }
        if (gamepad.x) {
            wobbleArmTargetAngle += armDegIncBig;
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
            om.sleep(300);
        }
        if (gamepad.b) {
            wobbleArmTargetAngle -= armDegIncBig;
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
            om.sleep(300);
        }

    }

    public void autoWobbleMotorVariable(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.y) {
            wobbleArmTargetAngle = 190.0;/* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE  - this is drop angle   */
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
            om.sleep(300);
        }
        if (gamepad.a) {
            wobbleArmTargetAngle = 65;/* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE - this is perpendicular before drop    */
//            wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
            om.sleep(300);
        }

        c = 1;//Coach note: what's this?  replaced by "pressAToContinue"?

    }

    public void setWobbleMotorPosition(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_right) {
//            wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));// Updated coach method
            om.sleep(300);
        }

    }

    public void setWobbleServoPos(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.y) {

            wobbleGoalPos += wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }
        if (gamepad.a) {

            wobbleGoalPos -= wobbleGrabInc;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }

    }

    public void pickUpWobble(BasicOpMode om) {




    }

    public void dropWobble(BasicOpMode om) {

        wobbleGoalArm.setPower(0.5);

        wobbleArmTargetAngle = 65.0;
        /* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE    */
//        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        while(Math.abs(wobbleGoalArm.getTargetPosition() - wobbleGoalArm.getCurrentPosition()) > 10){// alternate loop criteria but need variable for tolerances

//        while(wobbleGoalArm.isBusy()){// might not be robust -- take too long to settle and exit
            // do nothing but wait for arm to move within tolerance
            om.robotUG.driveTrain.robotNavigator(om);//replaces angleUnwrap (called in navigator)
            om.telemetry.addLine("WOBBLE GOAL DROP:");
            om.telemetry.addData("\tArm Target", "Goal Arm Target Angle (%.1f) degrees", om.robotUG.wobbleArm.wobbleArmTargetAngle);
            om.telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",om.robotUG.wobbleArm.getArmAngleDegrees());
            om.telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", om.robotUG.wobbleArm.armPower, om.robotUG.wobbleArm.wobbleGoalArm.getTargetPosition());
            om.telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", om.robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
            om.telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    om.robotUG.wobbleArm.wobbleGrabPos, om.robotUG.wobbleArm.wobbleReleasePos);
            om.telemetry.addData("\tServo Position", "Servo Pos (%.2f)",om.robotUG.wobbleArm.wobbleGoalServo.getPosition());
            om.telemetry.addLine("________________________________");
            om.telemetry.update();
        }
        wobbleGoalServo.setPosition(0.8);//this is a loose grip
        om.sleep(500);//might not need to be this long

        wobbleArmTargetAngle = 190.0;
        /* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE   to drop goal */

//        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        while(Math.abs(wobbleGoalArm.getTargetPosition() - wobbleGoalArm.getCurrentPosition()) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.robotUG.driveTrain.robotNavigator(om);//replaces angleUnwrap (called in navigator)
            om.telemetry.addLine("WOBBLE GOAL DROP:");
            om.telemetry.addData("\tArm Target", "Goal Arm Target Angle (%.1f) degrees", om.robotUG.wobbleArm.wobbleArmTargetAngle);
            om.telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",om.robotUG.wobbleArm.getArmAngleDegrees());
            om.telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", om.robotUG.wobbleArm.armPower, om.robotUG.wobbleArm.wobbleGoalArm.getTargetPosition());
            om.telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", om.robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
            om.telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    om.robotUG.wobbleArm.wobbleGrabPos, om.robotUG.wobbleArm.wobbleReleasePos);
            om.telemetry.addData("\tServo Position", "Servo Pos (%.2f)",om.robotUG.wobbleArm.wobbleGoalServo.getPosition());
            om.telemetry.addLine("________________________________");
            om.telemetry.update();

        }
        wobbleGoalServo.setPosition(0.4);// this is open grip without going max open
        om.sleep(500);//might not need to be this long
        wobbleArmTargetAngle = 25.0;// this is back near start but not all the way to avoid overshoot and impact
        /* LIFT ARM   */
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        while(Math.abs(wobbleGoalArm.getTargetPosition() - wobbleGoalArm.getCurrentPosition()) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.robotUG.driveTrain.robotNavigator(om);//replaces angleUnwrap (called in navigator)
            om.telemetry.addLine("WOBBLE GOAL DROP:");
            om.telemetry.addData("\tArm Target", "Goal Arm Target Angle (%.1f) degrees", om.robotUG.wobbleArm.wobbleArmTargetAngle);
            om.telemetry.addData("\tArm Angle", "Goal Arm Current Angle (%.2f) degrees",om.robotUG.wobbleArm.getArmAngleDegrees());
            om.telemetry.addData("\tMotor Variables", "Goal Arm Power (%.2f), Goal Arm Target (%d) counts", om.robotUG.wobbleArm.armPower, om.robotUG.wobbleArm.wobbleGoalArm.getTargetPosition());
            om.telemetry.addData("\tMotor Position", "Goal Arm Current Pos (%d) counts", om.robotUG.wobbleArm.wobbleGoalArm.getCurrentPosition());
            om.telemetry.addData("\tServo Variables", "Goal Grab (%.2f), Goal Release (%.2f)",
                    om.robotUG.wobbleArm.wobbleGrabPos, om.robotUG.wobbleArm.wobbleReleasePos);
            om.telemetry.addData("\tServo Position", "Servo Pos (%.2f)",om.robotUG.wobbleArm.wobbleGoalServo.getPosition());
            om.telemetry.addLine("________________________________");
            om.telemetry.update();

        }

    }
    /* -- COACH ADDITIONS: added some useful methods to simplify top level telemetry and commands
     *   - return ArmAngle in Degrees
     *   - convert ArmAngle in degrees to counts
     *   - notice how these remove redundant code form 'if' statements above and simplify telemetry
     *   - made a shutdown method for all hardware
     */
    public double getArmAngleDegrees(){
        return wobbleGoalArm.getCurrentPosition()/(MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO); //
    }
    public int angleToCounts(double angle){
        return (int) Math.round(angle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));//returns counts to use in motor
    }
    public void getTelemetry(BasicOpMode om){
        om.telemetry.addData("\tArm Angle", "Target = %.1f(deg.); Current = %.1f(deg.))", wobbleArmTargetAngle, getArmAngleDegrees());
        om.telemetry.addData("\tMotor Power", "Command= %.2f; Actual= %.2f;", armPower,wobbleGoalArm.getPower());
        om.telemetry.addData("\tMotor Position", "Target = %.1f(cnt); Current = %.1f(cnt)", wobbleGoalArm.getTargetPosition(),wobbleGoalArm.getCurrentPosition());
        om.telemetry.addData("\tServo Variables", "Grab(%.2f), Release(%.2f)", wobbleGrabPos, wobbleReleasePos);
        om.telemetry.addData("\tServo Position", "%.2f",wobbleGoalServo.getPosition());
        // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
        // Can add more lines as needed or max a BASIC and MAX TM option

    }
    public void shutdown(){// sort of redundant but confirms to common naming structure for use in HWMulti
        wobbleGoalArm.setZeroPowerBehavior(FLOAT);
        wobbleGoalArm.setPower(0.0);//could result in crash of arm when hitting stop
        // believe best to remove power
        wobbleGoalServo.setPosition(0.0);//stop gripping
    }
}
