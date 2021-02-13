package UltimateGoal_RobotTeam.HarwareConfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

//import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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
    public double wobbleGoalPos = 0.9;// tight servo grabbing
    public double wobbleGrabInc = 0.1;
    public double wobbleGrabPos = 0.5;
    public double wobbleReleasePos = 0.4;// this is enough to release goal but not full open
    public int wobbleArmTarget = 0;//KS: this can be deleted it's redundant when there's a conversion method
    public double wobbleArmTargetAngle = 0.0;// this needs be a double - might want decimal degrees
    public int armDegInc = 1;
    public int armDegIncBig = 25; // KS - increments could be candidates for hashmap parameter if need to carefully set angles
    public double armPower = 0.25;
    public double powerInc = 0.05;
    public double armPowerHold = 0.7;
    public final double ARM_GEAR_RATIO = 24/15; //Coach Note: updated value is 24.0/15.0, commands were updated but don't make sense
            // Drop position of 190 > 180 doesn't physically make sense when observing robot
            // Error might be integer division in input 24/15 = 1.0 vs. 24.0/15.0 = 1.6 - need to investigate in a test OpMode
            // 24 = motor motion for 15 of input arm motion output * 24/15 = motor OR motor * 15/24 = output
            // Also Updated for Java constant style guide
    private final double MOTOR_DEG_TO_COUNT = 1440.0/360.0; // Coach Note: since motor is part of this HW can make local
        // don't need to change with any global parameter and therefore avoids having to pass in om.cons for final value

//    public int c = 0; //temporary variable use to prevent us from choosing how we want to progress before we drop the wobble goal
    /* Parameters used for offline code for locating arm
     *
     */
    public final double ARM_LENGTH = 12.5;
    public final double ARM_X = 8.0;//location Right/Left on robot for arm pivot
    public final double ARM_Y = -6.5;//location FWD/BACK on robot for arm pivot
    public final double ARM_INIT_ANGLE_DEG = 35.0;

    public WobbleArm(BasicOpMode om, boolean tm)  {
        if(tm) {
            om.telemetry.addData("Wobble Arm", " Initializing...");
            om.telemetry.update();
//            wobbleGoalServo = new Servo();wobbleGoalArm = new DcMotor();wobbleGoalArm.timeStep = om.timeStep * (0.5);//NEEDED FOR OFFLINE
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
            om.sleep(300);
        }
        if (gamepad.right_bumper) {
            armPower += powerInc;
            om.sleep(300);
        }

        wobbleGoalArm.setPower(armPower);

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

//        c = 1;//Coach note: what's this?  replaced by "pressAToContinue"?

    }

    public void setWobbleMotorPosition(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_right) {
//            wobbleGoalArm.setTargetPosition(wobbleArmTarget);
            wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));// Updated coach method
            om.sleep(300);
        }

    }

    public void moveWobbleServo(Gamepad gamepad, BasicOpMode om) {

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

    public void setWobbleServoPos(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.y) {

            wobbleGoalPos = 0.9;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }
        if (gamepad.a) {

            wobbleGoalPos = 0.25;
            wobbleGoalServo.setPosition(wobbleGoalPos);
            om.sleep(250);
        }

    }

    public void setWobbleGoalArmDown(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_down){
            wobbleArmTargetAngle = 190.0;/* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE - this is perpendicular to grab    */
            wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));// Updated coach method
            om.sleep(300);
        }

    }

    public void setWobbleGoalArmUp(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_up){
            wobbleArmTargetAngle = 70;// angle for Wobble Goal over field wall || was 80
            wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));// Updated coach method
            om.sleep(300);
        }

    }

    public void setWobbleGoalArmOverWall(Gamepad gamepad, BasicOpMode om) {

        if (gamepad.dpad_right){
            wobbleArmTargetAngle = 90;// angle for Wobble Goal over wall
            wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));// Updated coach method
            om.sleep(300);
        }

    }

    public void dropWobble(BasicAuto om) {

        wobbleGoalArm.setPower(0.5);

        wobbleArmTargetAngle = 65.0;
        /* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE    */
//        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        int armPos = wobbleGoalArm.getCurrentPosition();
        int count = 0;
        om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS

        while(Math.abs(wobbleGoalArm.getTargetPosition() - armPos) > 10){// alternate loop criteria but need variable for tolerances

//        while(wobbleGoalArm.isBusy()){// might not be robust -- take too long to settle and exit
            // do nothing but wait for arm to move within tolerance
            om.telemetry.addLine("WOBBLE GOAL DROP: PREP");
            om.telemetry.addData("Loop count", " %d",count);

            armPos = getTelemetry(om);
            om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS

            om.telemetry.addLine("________________________________");
            om.telemetry.update();
            count +=1;

        }
        wobbleGoalServo.setPosition(0.8);//this is a loose grip
        if(!om.testModeActive) {
            om.sleep(500);//might not need to be this long
        }
        wobbleArmTargetAngle = 190.0;
        /* UPDATED ABOVE ANGLE FOR GEAR RATIO UPDATE   to drop goal */

//        wobbleArmTarget = (int) Math.round(wobbleArmTargetAngle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));// KS added om.cons & commented
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        armPos = wobbleGoalArm.getCurrentPosition();
        om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS
        count = 0;
        while(Math.abs(wobbleGoalArm.getTargetPosition() - armPos) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.telemetry.addLine("WOBBLE GOAL DROP: DROP");
            om.telemetry.addData("Loop count", " %d",count);

            armPos = getTelemetry(om);
            om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS

            om.telemetry.addLine("________________________________");
            om.telemetry.update();
            count +=1;
        }
        wobbleGoalServo.setPosition(0.4);// this is open grip without going max open
        if(!om.testModeActive) {
            om.sleep(500);//might not need to be this long
        }
        om.haveBlueWobble1 = false;// wobble goal dropped
        om.haveBlueWobble2 = false;// wobble goal dropped
        om.haveRedWobble1 = false;// wobble goal dropped
        om.haveRedWobble2 = false;// wobble goal dropped

        wobbleArmTargetAngle = 25.0;// this is back near start but not all the way to avoid overshoot and impact
        /* LIFT ARM   */
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        armPos = wobbleGoalArm.getCurrentPosition();
        om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS
        count = 0;
        while(Math.abs(wobbleGoalArm.getTargetPosition() - armPos) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.telemetry.addLine("WOBBLE GOAL DROP: RAISE");
            om.telemetry.addData("Loop count", " %d",count);

            armPos = getTelemetry(om);
            om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS
            om.telemetry.addLine("________________________________");
            om.telemetry.update();
            count +=1;

        }

        om.robotUG.driveTrain.setMotorPower(0.2);

        wobbleArmTargetAngle = 0.0;// this is back near start but not all the way to avoid overshoot and impact
        /* LIFT ARM   */
        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        armPos = wobbleGoalArm.getCurrentPosition();
        om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS
        count = 0;
        while(Math.abs(wobbleGoalArm.getTargetPosition() - armPos) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.telemetry.addLine("WOBBLE GOAL DROP: RAISE");
            om.telemetry.addData("Loop count", " %d",count);

            armPos = getTelemetry(om);
            om.robotUG.driveTrain.robotNavigator(om);//NEEDED FOR OFFLINE CODE TO UPDATE POSITIONS
            om.telemetry.addLine("________________________________");
            om.telemetry.update();
            count +=1;

        }

    }

    public void grabWobble(BasicOpMode om) {

        wobbleGoalArm.setPower(0.5);
        wobbleArmTargetAngle = 190.0;

        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        while(Math.abs(wobbleGoalArm.getTargetPosition() - wobbleGoalArm.getCurrentPosition()) > 10){// alternate loop criteria but need variable for tolerances

            // do nothing but wait for arm to move within tolerance
            om.robotUG.driveTrain.robotNavigator(om);//replaces angleUnwrap (called in navigator)
            om.telemetry.addLine("WOBBLE GOAL DROP: PREP");
            getTelemetry(om);

            om.telemetry.addLine("________________________________");
            om.telemetry.update();
        }
        wobbleGoalServo.setPosition(0.9);//this is a good grip
        om.sleep(500);//might not need to be this long

        wobbleArmTargetAngle = 65.0;

        wobbleGoalArm.setTargetPosition(angleToCounts(wobbleArmTargetAngle));
        while(Math.abs(wobbleGoalArm.getTargetPosition() - wobbleGoalArm.getCurrentPosition()) > 10){// alternate loop criteria but need variable for tolerances
            // do nothing but wait for arm to move within tolerance
            om.robotUG.driveTrain.robotNavigator(om);//replaces angleUnwrap (called in navigator)
            om.telemetry.addLine("WOBBLE GOAL DROP: DROP");
            getTelemetry(om);
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
        return wobbleGoalArm.getCurrentPosition()/(MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO); //return arm angle in degrees based on motor counts
          }
    public double convertArmAngleDegrees(int armCounts){
        return armCounts/(MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO); //return arm angle in degrees based on motor counts
    }
    public int angleToCounts(double angle){
        return (int) Math.round(angle * (MOTOR_DEG_TO_COUNT * ARM_GEAR_RATIO));//returns counts to use in motor
    }
    public int getTelemetry(BasicOpMode om){
        int pos = wobbleGoalArm.getCurrentPosition();//Added to aid offline and reduce calls to calculate motor position
        om.telemetry.addData("\tMotor Position", "Target = %d(cnt); Current = %d(cnt)", wobbleGoalArm.getTargetPosition(),pos);
        om.telemetry.addData("\tArm Angle", "Target = %.1f(deg.); Current = %.1f(deg.))", wobbleArmTargetAngle, convertArmAngleDegrees(pos));
        om.telemetry.addData("\tMotor Power", "Command= %.2f; Actual= %.2f;", armPower,wobbleGoalArm.getPower());
        om.telemetry.addData("\tServo Variables", "Grab(%.2f), Release(%.2f)", wobbleGrabPos, wobbleReleasePos);
        om.telemetry.addData("\tServo Position", "%.2f",wobbleGoalServo.getPosition());
        // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
        // Can add more lines as needed or max a BASIC and MAX TM option
        return pos;
    }
    public void shutdown(){// sort of redundant but confirms to common naming structure for use in HWMulti
        wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleGoalArm.setPower(0.0);//could result in crash of arm when hitting stop
        // believe best to remove power
        wobbleGoalServo.setPosition(0.0);//stop gripping
    }
}
