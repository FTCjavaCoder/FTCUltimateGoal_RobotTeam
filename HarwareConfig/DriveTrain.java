package UltimateGoal_RobotTeam.HarwareConfig;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

//import OfflineCode.OfflineHW.BNO055IMU;
//import OfflineCode.OfflineHW.DcMotor;
//import OfflineCode.OfflineHW.JustLoggingAccelerationIntegrator;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.OpModes.BasicOpMode;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;
import UltimateGoal_RobotTeam.Utilities.FieldLocation;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;
import UltimateGoal_RobotTeam.Utilities.PursuitPoint;

public class DriveTrain {

    /**
     * NEW FOR ULTIMATE GOAL - Coach File - can be used by team or HardwareRobot can still be used
     * 11/1/20 - this file now configures a DriveTrain just like the driving portion of HardwareRobot
     * DriveTrain will contain IMU and navigation
     * DriveTrain contains all the dirving (Auto and TelOp) methods for those functions
     */
    /* Public OpMode members. */
    public DcMotor frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft    = null;
    public DcMotor  backRight   = null;
    public BNO055IMU imu = null;

    // Global variables for methods that have moved to this class
    int targetPos[] = new int[4];

    public enum moveDirection {FwdBack, RightLeft, Rotate}

    public double clockwise =0;
    public double forwardDirection =0;
    public double rightDirection =0;
    public double verticalDirection = 0;
    public double clockwiseDirection =0;
    public double counterclockwiseDirection = 0;
    public double slideDirection =0;

    public double priorAngle = 0;
    public double offset = 0;
    public double robotHeading = 0;

    public int currentPos[] = new int[4];
    public int priorPos[] = new int[4];
    public double distanceTraveled = 0;

    private boolean reduceDrivingPower = false; // << Private property to know if reduced Driving Power should apply
    /* WHEN IS ABOVE USED? */
    /** items below are for pure pursuit and robot navigator
     *
     */
    public int flPrevious;
    public int frPrevious;
    public int brPrevious;
    public int blPrevious;


    public double robotAngle;
    public double robotX;
    public double robotY;
    public double fieldX;
    public double fieldY;
    public double fieldAngle;

    public double targetHeading;

    public int countDistance = 0;

    public FieldLocation robotLocationV1 = new FieldLocation(0,0,0);
    public FieldLocation robotFieldLocation = new FieldLocation(0,0,0);


    public PursuitPoint targetPoint = new PursuitPoint(0,0);
    /* End Pure Pursuit Items */

    /* local OpMode members. */
    public Orientation angles;

    /** CONSTRUCTORS
     * Taking the approach that when constructing robot all the initialization items can be run
     * Approach is that null Robot existing in BasicOpMode and then each OpMode constructs the Robot it needs
     * Methods within BasicAuto to be re-used would need to pass the Robot to have the correct robot config
     * This enables having similar code for different configs
     * @param om: this is the OpMode that is constructing the
     * @param tm: this is the testMode boolean, if in testMode (Offline) then need to create new instances
     *          if NOT in testMode (real robot) need to map null objects
     * Believe this can be made such that testMode robot is constructed on it's own and eliminates this boolean
     * implementint testMode as a separate constructor
     */
    public DriveTrain(BasicOpMode om, boolean tm) {
        if(tm) {
            om.telemetry.addData("ERROR: ", "Initializing DriveTrain in TestMode...");

            /** ONLY USED IN OFFLINE TEST MODE
             *
             */

//            // Define and Initialize Motors
//            frontLeft = new DcMotor();
//            frontRight = new DcMotor();
//            backLeft = new DcMotor();
//            backRight = new DcMotor();
//
//            imu = new BNO055IMU();
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();//Need help on enclosing class
//            imu.initialize(parameters);
        }
        else {
            om.telemetry.addData("Status: ", "Initializing DriveTrain ...");
            // Define and Initialize Motors
            frontLeft = om.hardwareMap.get(DcMotor.class, "motor_fl");
            frontRight = om.hardwareMap.get(DcMotor.class, "motor_fr");
            backLeft = om.hardwareMap.get(DcMotor.class, "motor_bl");
            backRight = om.hardwareMap.get(DcMotor.class, "motor_br");

            //Define all installed sensors


            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = om.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

             // INITIALIZE DRIVE TRAIN MOTORS
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Reset all motor encoders
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Indicate initialization complete and provide telemetry
            om.telemetry.addData("Status: ", "Drive Train Initialized");

        }

    }

    /** METHODS TO INITIALIZE SPECIFIC HW
     *
     * @param om: specific OpMode calling the function - only needed for the sleep which seems out of place
     */
    public void initIMU(BasicOpMode om) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//This line calls the angles from the IMU

        offset = angles.firstAngle; //Determine initial angle offset 
        priorAngle = offset; //set prior angle for unwrap to be initial angle 
        robotHeading = angles.firstAngle - offset; //robotHeading to be set at 0 degrees to start 

        // Should there be an input to set the angle or should that be for the field angle?

        om.sleep(100);
    }
    /** METHODS TO COMPELTE CALCULATIONS NEEDED FOR OTHER DRIVIGN METHODS
     *
     */

    public void angleUnWrap() {

        double deltaAngle;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//This line calls the angles from the IMU
        deltaAngle = -(angles.firstAngle - priorAngle);// Determine how much the angle has changed since we last checked teh angle
        if (deltaAngle > 180) {//This is IF/THEN for the unwrap routine
            robotHeading += deltaAngle - 360;//Decrease angle for negative direction //rotation
        } else if (deltaAngle < -180) {
            robotHeading += deltaAngle + 360;//increase angle for positive direction //rotation 
        } else {
            robotHeading += deltaAngle;//No wrap happened, don't add any extra rotation
        }
        priorAngle = angles.firstAngle;//Update the latest measurement to be //priorAngle for the next time we call the method

    }

    public double calcSteeringPowerIMU(double angleWanted, BasicAuto om) {

        angleUnWrap();
        double steerPower = (angleWanted - robotHeading) * om.cons.STEERING_POWER_GAIN;

        double clippedSteering = -1.0 * (Range.clip(steerPower, -om.cons.STEERING_POWER_LIMIT, om.cons.STEERING_POWER_LIMIT) );

        return clippedSteering;
    }

    public void calcDistanceIMU(int[] driveDirection, BasicAuto om) {
        int deltaPos[] = new int[4];
        int adjustedPos[] = new int[4];
        double rotationOffset;
        double incrementalDistance;

        for(int i=0; i < 4; i++){

            deltaPos[i] = currentPos[i] - priorPos[i];// remove global variables for prior and current by passing to method
        }

        rotationOffset = (deltaPos[0] + deltaPos[1] + deltaPos[2] + deltaPos[3] ) / 4;//not used

        for(int i = 0; i < 4; i++){

            adjustedPos[i] = (int) Math.round(deltaPos[i] - rotationOffset); //not used

        }

        incrementalDistance = ((deltaPos[0] * driveDirection[0]) + (deltaPos[1] * driveDirection[1]) + (deltaPos[2] * driveDirection[2]) + (deltaPos[3] * driveDirection[3])) / 4;

        distanceTraveled += incrementalDistance / om.cons.DEGREES_TO_COUNTS_40_1 / om.cons.ROBOT_INCH_TO_MOTOR_DEG;//adjForward & adjRight used in the desired distance calculation
        //remove global variable by returning distance traveled from method, pass in current distance, retrun distance + increment
        priorPos = currentPos;

        om.telemetry.addData("Motor Movement", "FL (%d) FR (%d) BR (%d) BL (%d)", deltaPos[0], deltaPos[1], deltaPos[2], deltaPos[3]);
        om.telemetry.addData("Robot Movement", "Incremental: (%.2f) Total: (%.2f)", incrementalDistance / om.cons.DEGREES_TO_COUNTS_40_1 / om.cons.ROBOT_INCH_TO_MOTOR_DEG, distanceTraveled);
        om.telemetry.update();

    }
    public int[] getMotorPosArray() {

        int[] currentPos = new int[4];

        currentPos[0]= frontLeft.getCurrentPosition(); //FL
        currentPos[1]= frontRight.getCurrentPosition(); //FR
        currentPos[2]= backRight.getCurrentPosition(); //BR
        currentPos[3]= backLeft.getCurrentPosition(); //BL

        return currentPos;
    }

    public void setMotorPower(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

    }

    public void setMotorPowerArray(double[] power) {

        frontLeft.setPower(power[0]);
        frontRight.setPower(power[1]);
        backRight.setPower(power[2]);
        backLeft.setPower(power[3]);

    }

    public boolean targetPosTolerence(BasicAuto om) {

        int countTol = 0;
        Boolean motorFinish = false;
        int[] motorPos = new int[4];

        motorPos = getMotorPosArray();
        for (int i = 0; i < 4; i++) {

            if (om.cons.MOVE_TOL >= Math.abs(motorPos[i] - targetPos[i])) {

                countTol += 1;

                if (countTol == 1) {// was 4 also 1

                    motorFinish = true;

                }
            }

        }

        return motorFinish;

    }

    public void robotNavigator(BasicOpMode om){

        int flCount = frontLeft.getCurrentPosition();
        int frCount = frontRight.getCurrentPosition();
        int brCount = backRight.getCurrentPosition();
        int blCount = backLeft.getCurrentPosition();

        int deltaFL = flCount - flPrevious;
        int deltaFR = frCount - frPrevious;
        int deltaBR = brCount - brPrevious;
        int deltaBL = blCount - blPrevious;


        // Update to use IMU angle so "robotHeading" is used to update fieldAngle

        // Simplify by averaging all distance in correct frame
        double robotXInc = ((-deltaFL + deltaFR + deltaBR - deltaBL)/4)/
                (om.cons.DEGREES_TO_COUNTS_60_1*om.cons.ROBOT_INCH_TO_MOTOR_DEG*om.cons.adjForward);
        double robotYInc = -((-deltaFL - deltaFR + deltaBR + deltaBL)/4)/
                (om.cons.DEGREES_TO_COUNTS_60_1*om.cons.ROBOT_INCH_TO_MOTOR_DEG*om.cons.adjRight);
        robotX += robotXInc;
        robotY += robotYInc;
/** robotX & robotY are not used else where - navigator tracks field coordinates
 */
//Coordinate transformation to take robot body coordinates to field reference frame - depends on IMU angle
//Angle reference from field to robot is negative angle in CW = + robot frame, uses + = CCW {IMU frame & field frame}
        fieldAngle = -robotHeading;//IMU field angle should be same sign as used for theta in rotation matrix, why returning "-"

        double fieldXInc = (robotXInc*Math.cos(Math.toRadians(fieldAngle))) -
                (robotYInc*Math.sin(Math.toRadians(fieldAngle)));//flipped sign on Ry term
        double fieldYInc = (robotXInc*Math.sin(Math.toRadians(fieldAngle))) +
                (robotYInc*Math.cos(Math.toRadians(fieldAngle)));//flipped sign on Rx term
        fieldX += fieldXInc;
        fieldY += fieldYInc;

        //Update robot FieldLocation for X,Y, angle
        robotFieldLocation.setLocation(fieldX,fieldY,fieldAngle);
        /** should robotFieldLocation be returned or should deltas and angle be returned??
         */

        /** previous values can be handled outside navigator or passed in
         */
        flPrevious = flCount;
        frPrevious = frCount;
        brPrevious = brCount;
        blPrevious = blCount;
    }

    /**
     * METHODS BELOW MOVED FROM pursuitMath and pursuitPath
     */
    public PursuitPoint findPursuitPoint(ArrayList<PursuitPoint> inputPoints, FieldLocation robot, double radius) {

//        PursuitPoint currentPoint = inputPoints.get(0); // issue with tracking the start point as in scope
        //set line starting point as default for starting to follow ... when robot gets lost it heads to first line point

        PursuitPoint currentPoint = null;



//        PursuitPoint robotOffsetPoint = new PursuitPoint(robot.x,robot.y);
//        FieldLocation robotOffset = new FieldLocation(0,0,robot.theta);//create field location to pass robot angle as offset at (0,0)

        ArrayList<PursuitPoint> targetPoints = new ArrayList<>();

        double currentAngle = 1000;

// Search all lines for the valid intersections with the robot radius and that line withi its endpoints
//        robotOffsetPoint.offset(robot.x, robot.y);//offset point for robot to be (0,0)
//        currentPoint.offset(robot.x,robot.y);
        /**
         * Issue with using initial point because it's on a line and angle can be less than other points
         * Might need to use distance check as well (> radius)
         * Seems to run fine w/o this guarantee of a point - as long as path starts at robot
         */
//        targetPoints.add(currentPoint);//Ensure a minimum of 1 point to search
//        ArrayList<PursuitPoint> PointList = new ArrayList<>();

        for(int i = 0; i<inputPoints.size()-1;i++) {

//            pl.offset(robot.x, robot.y);//offset points for robot to be (0,0) for circle math
            PursuitPoint first = new PursuitPoint(inputPoints.get(i).x,inputPoints.get(i).y);
            PursuitPoint second = new PursuitPoint(inputPoints.get(i+1).x,inputPoints.get(i+1).y);

            //offset lines for robot being moved to (0,0);
            first.offset(robot.x,robot.y);
            second.offset(robot.x,robot.y);

            PursuitLines pl = new PursuitLines(first,second);

            //avoid infinite or zero slope lines
            if(Math.abs(pl.x2 - pl.x1)<0.01){
                pl.x2 = pl.x1+0.1;
            }
            if(Math.abs(pl.y2 - pl.y1)<0.01){
                pl.y2 = pl.y1+0.1;
            }

            pl.calcSlope();
            //Determine quadratic formula constants
            double constb =2.0*pl.b*pl.slope;
            double const4ac =4.0 * ( (1+Math.pow(pl.slope,2)) * (Math.pow(pl.b,2) - Math.pow(radius,2)) );
            double const2a=2.0 * (1+Math.pow(pl.slope,2));

            double Xpos;double Xneg;double Ypos;double Yneg;
            //to compute quadratic - because sqrt, could be imaginary, so if statement puts imaginary values off the line
            if(const4ac > Math.pow(constb, 2)) {
                Xpos = -1000;
                Xneg = -1000;
            }
            else {

                Xpos = ( -constb + Math.sqrt(Math.pow(constb, 2) - const4ac) ) / const2a;
                Xneg = ( -constb - Math.sqrt(Math.pow(constb, 2) - const4ac) ) / const2a;

            }
            Ypos = Xpos * pl.slope + pl.b;
            Yneg = Xneg * pl.slope + pl.b;

            PursuitPoint PosPoint = new PursuitPoint(Xpos, Ypos);
            PursuitPoint NegPoint = new PursuitPoint(Xneg, Yneg);



            //Offset back into the actual coordinates - robot != (0,0)
            PosPoint.offset(-robot.x,-robot.y);
            NegPoint.offset(-robot.x,-robot.y);

//            Xpos -= robot.x;
//            Xneg -= robot.x;
//
//            Ypos -= robot.y;
//            Yneg -= robot.y;

            pl.offset(-robot.x,-robot.y);
            double minX;// = pl.x1 < pl.x2 ? pl.x1 : pl.x2;
            double maxX;// = pl.x1 > pl.x2 ? pl.x1 : pl.x2;
            if(pl.x1 < pl.x2){minX = pl.x1;}
            else{ minX = pl.x2;}
            if(pl.x1 > pl.x2){maxX = pl.x1;}
            else{ maxX = pl.x2;}
//            double minX = -30.0;
//            double maxX = 0.0;

            //validate that intersection is within line endpoints
            if ((PosPoint.x >= minX) && (PosPoint.x <= maxX)) {
                targetPoints.add(PosPoint);//include if in boundaries
            }


            //validate that intersection is within line endpoints
            if ((NegPoint.x >= minX) && (NegPoint.x <= maxX)) {
                targetPoints.add(NegPoint);//include if in boundaries
            }

        }
        //Search list of intersection points  for closest angle to robot
        for(PursuitPoint testPoint:targetPoints){
            double pointAngle = findAbsAngle(testPoint,robot);
            if(pointAngle <= currentAngle){
                currentPoint = testPoint;
                currentAngle = pointAngle;
            }
        }

        return currentPoint;
    }
    public double selectiveWrap(double angle, double angleCompare) {
        // keep wrap point away from angleCompare +/- 90
        if(((angleCompare % 360) > 90) || ((angleCompare % 360) < -90)){
            //Wrap at 0/360 deg.
            if (angle > 0) {//This is IF/THEN for the wrap routine
                return angle  % 360 ;//Decrease angle for negative direction //rotation
            }
            else {
                return (angle + 360) % 360;//increase angle for positive direction //rotation 
            }
        }
        else {
            //wrap at 180/-180
            if (angle > 0) {//This is IF/THEN for the wrap routine
                return ((angle + 180) % (360)) - 180;}
            else{
                return ((angle - 180) % (360)) + 180;
            }
        }
    }
    public double findDistance(PursuitPoint point, PursuitPoint robot){
        return Math.sqrt(Math.pow(point.x - robot.x,2) + Math.pow(point.y - robot.y,2));//distance equation
    }
    public double findAbsAngle(PursuitPoint point, FieldLocation robot){
        double angle = Math.atan2((point.y - robot.y) , (point.x - robot.x))*180/Math.PI;//angle in radians converted to degrees, wrapped +/-180 deg angle
        // issue is that atan2 wraps goes form 179 to - 179 as it crosses 180 - can't unwrap with typical algorithm don't have change to track
        // robot field location could be wrapped because could be greater than 180 or 360
        // when atan2 wraps the output then the pursuit path will select the opposite side of the circle

        angle = Math.abs(selectiveWrap(angle, robot.theta) - selectiveWrap(robot.theta, robot.theta));// wrap to +/-180 subtract robot field location and take absolute value, both in field frame
        // keep pursuit point in field frame, once selected it is passed to setRobotAngle which uses robot frame output
        return angle;
    }
    public ArrayList<PursuitPoint> findIntersection(PursuitLines pl, double radius, PursuitPoint robot){
        /** Note being used - might have errors
         * Compare against findPursuitPoint
         */
        ArrayList<PursuitPoint> PointList = new ArrayList<>();
        //avoid infinite or zero slope lines
        if(Math.abs(pl.x2 - pl.x1)<0.01){
            pl.x2 = pl.x1+0.01;
        }
        if(Math.abs(pl.y2 - pl.y1)<0.01){
            pl.y2 = pl.y1+0.01;
        }
        double minX = pl.x1 < pl.x2 ? pl.x1 : pl.x2;
        double maxX = pl.x1 > pl.x2 ? pl.x1 : pl.x2;
        //Determine quadratic formula constants
        double constb =2.0*pl.b*pl.slope;
        double const4ac =4.0*((1+Math.pow(pl.slope,2))*(Math.pow(pl.b,2)-Math.pow(radius,2)));
        double const2a=2.0*(1+Math.pow(pl.slope,2));

        //to compute quadratic - because sqrt, could be NaN
        try {

            double Xpos = ( -constb + Math.sqrt(Math.pow(constb, 2) - const4ac) )/ const2a;
            double Ypos = Xpos * pl.slope + pl.b;

            //validate that intersection is within line endpoints
            if ((Xpos >= pl.x1) && (Xpos <= pl.x2)) {
                PointList.add(new PursuitPoint(Xpos, Ypos));//include if in boundaries
            }

            double Xneg = ( -constb - Math.sqrt(Math.pow(constb, 2) - const4ac) )/ const2a;
            double Yneg = Xneg * pl.slope + pl.b;
            //validate that intersection is within line endpoints
            if ((Xneg >= minX) && (Xneg <= maxX)) {
                PointList.add(new PursuitPoint(Xneg, Yneg));//include if in boundaries
            }

        }catch(Exception e){

        }

        return PointList;

    }
    public void setTargetAngle(PursuitPoint target, FieldLocation field){
        double angle = -Math.atan2(target.y- field.y, target.x- field.x) * 180/Math.PI;
        /** negative sign for the change in rotation sign convention + = CW for steering
         * NOTE SIGN CONVENTION
         */

//Need to maintain unwrapped target angle ... unwraps delta angle - an issue if the target changes by 180?0
        double deltaAngle = angle - targetHeading;// difference between robot center to target point and robot heading
        if (deltaAngle > 180) {//This is IF/THEN for the unwrap routine
            targetHeading += deltaAngle - 360;//Decrease angle for negative direction //rotation
        } else if (deltaAngle < -180) {
            targetHeading += deltaAngle + 360;//increase angle for positive direction //rotation 
        } else {
            targetHeading += deltaAngle;//No wrap happened, don't add any extra rotation
        }
    }

    /** METHODS FOR AUTONOMOIUS DRIVING
     *
     */

    public void IMUDriveFwdRight(moveDirection moveType, double distanceInch, double targetAngle, String step, BasicAuto om) {
        int countDistance = 0;
        int[] driveDirection = new int[4];
        int[] startPos = new int[4];
        boolean motorsDone = false;
        double[] error = new double[4];
        double[] setPower = new double[4];
        double scaledDistance = 0;

        setMotorPower(0);

        distanceTraveled = 0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch (moveType) {

            case FwdBack:
                scaledDistance = (distanceInch * om.cons.adjForward);

                driveDirection[0] = -1;// FL
                driveDirection[1] = +1;// FR
                driveDirection[2] = +1;// BR
                driveDirection[3] = -1;// BL
                break;

            case RightLeft:
                scaledDistance = (distanceInch * om.cons.adjRight);
                driveDirection[0] = -1;// FL
                driveDirection[1] = -1;// FR
                driveDirection[2] = +1;// BR
                driveDirection[3] = +1;// BL
                break;

            default:

                driveDirection[0] = 0;// FL
                driveDirection[1] = 0;// FR
                driveDirection[2] = 0;// BR
                driveDirection[3] = 0;// BL
        }

        currentPos = getMotorPosArray();

        priorPos = currentPos;

        calcDistanceIMU(driveDirection, om);

        double steeringPower = calcSteeringPowerIMU(targetAngle, om);

        om.updateIMU();

        double prePower[] = new double[4];

        for (int i = 0; i < 4; i++) {

            prePower[i] = Range.clip(((scaledDistance - distanceTraveled) * driveDirection[i]) * om.cons.POWER_GAIN, -om.cons.DRIVE_POWER_LIMIT, om.cons.DRIVE_POWER_LIMIT) + steeringPower;

        }

        double maxLeft = Math.max(Math.abs(prePower[0]), Math.abs(prePower[3]));
        double maxRight = Math.max(Math.abs(prePower[1]), Math.abs(prePower[2]));
        double max = Math.max(maxLeft, maxRight);

        if (om.cons.DRIVE_POWER_LIMIT >= max) {

            max = 1;
        }

        for (int i = 0; i < 4; i++) {

            prePower[i] = Range.clip(Math.abs(prePower[i] / max), om.cons.DRIVE_POWER_MINIMUM, om.cons.DRIVE_POWER_LIMIT) * Math.signum(prePower[i]);

        }

        setMotorPowerArray(prePower);

        while ((Math.abs(scaledDistance - distanceTraveled) > om.cons.IMU_DISTANCE_TOL) && (om.opModeIsActive() || om.testModeActive)) {

            currentPos = getMotorPosArray();

            calcDistanceIMU(driveDirection, om);

            steeringPower = calcSteeringPowerIMU(targetAngle, om);

            om.updateIMU();

            for (int i = 0; i < 4; i++) {

                prePower[i] = Range.clip(((scaledDistance - distanceTraveled) * driveDirection[i]) * om.cons.POWER_GAIN, -om.cons.DRIVE_POWER_LIMIT, om.cons.DRIVE_POWER_LIMIT) + steeringPower;

            }

            maxLeft = Math.max(Math.abs(prePower[0]), Math.abs(prePower[3]));
            maxRight = Math.max(Math.abs(prePower[1]), Math.abs(prePower[2]));
            max = Math.max(maxLeft, maxRight);

            if (om.cons.DRIVE_POWER_LIMIT >= max) {

                max = 1;
            }

            for (int i = 0; i < 4; i++) {

                //*************** MODIFIED POWER CLIP FOR setPower TO HAVE DRIVE_POWER_MINIMUM ************************************

//                prePower[i] = prePower[i] / max;//NOT LIMITED TO DRIVE_POWER_MINIMUM
                setPower[i] = Math.signum(prePower[i]) * Range.clip(Math.abs(prePower[i] / max), om.cons.DRIVE_POWER_MINIMUM, om.cons.DRIVE_POWER_LIMIT);
                //*************** MODIFIED POWER CLIP FOR setPower TO HAVE DRIVE_POWER_MINIMUM ************************************
            }
            //*************** MODIFIED TO setPower  ************************************
            setMotorPowerArray(setPower);
            //*************** MODIFIED TO setPower  ************************************

            //*************** MODIFIED TELEMETRY FOR DEBUGGING ************************************
            om.telemetry.addData("Driving: ", step);
//            om.telemetry.addData("Motor Commands: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
//                    targetPos[0], targetPos[1],targetPos[2],targetPos[3]);
            om.telemetry.addData("Robot Heading: ", " Desired: %.2f, Actual: %.2f", targetAngle, robotHeading);
            om.telemetry.addData("Motor Counts: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
                    currentPos[0], currentPos[1], currentPos[2], currentPos[3]);
            om.telemetry.addData("Motor Power: ", "FL (%.2f) FR (%.2f) BR (%.2f) BL (%.2f)",
                    setPower[0], setPower[1], setPower[2], setPower[3]);
            om.telemetry.addData("Steering ", "Power: %.2f, Gain: %.3f", steeringPower, om.cons.STEERING_POWER_GAIN);
            om.telemetry.addData("Distance Moved: ", "%.2f", distanceTraveled);
            om.telemetry.addData("Time: ", om.runtime);
            om.telemetry.update();
        }

        setMotorPower(0);

        angleUnWrap();
        om.updateIMU();

        om.telemetry.addData("COMPLETED Driving: ", step);
//        om.telemetry.addData("Motor Commands: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
//                targetPos[0], targetPos[1],targetPos[2],targetPos[3]);
        om.telemetry.addData("Robot Heading: ", " Desired: %.2f, Actual: %.2f", targetAngle, robotHeading);
        om.telemetry.addData("Motor Counts: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backRight.getCurrentPosition(), backLeft.getCurrentPosition());
        om.telemetry.addData("Distance Moved: ", "%.2f", distanceTraveled);
        om.telemetry.addData("Time: ", om.runtime);
        om.telemetry.addLine("=========================================");
        om.telemetry.update();
        //*************** MODIFIED TELEMETRY FOR DEBUGGING ************************************

        if (om.cons.doRotateMethod == 1) {
            IMUDriveRotate(targetAngle, "Post move Rotate", om);//********NEW*********
        }
    }

    public void IMUDriveRotate(double targetAngle, String step, BasicAuto om) {
        int countDistance = 0;
        int[] driveDirection = new int[4];
        int[] startPos = new int[4];
        boolean motorsDone = false;
        double[] error = new double[4];
        double[] setPower = new double[4];
        double prePower[] = new double[4];
        double scaledDistance = 0 ;

        setMotorPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveDirection[0] = -1;// FL
        driveDirection[1] = -1;// FR
        driveDirection[2] = -1;// BR
        driveDirection[3] = -1;// BL
        //*************** ADDED CURRENT POSITION CALCULATION  ************************************

        currentPos = getMotorPosArray();
        //*************** ADDED CURRENT POSITION CALCULATION  ************************************

        angleUnWrap();
        om.updateIMU();

        for(int i = 0; i < 4; i++){

            prePower[i] = driveDirection[i] * (targetAngle - robotHeading) * om.cons.ROTATE_POWER_GAIN;

            setPower[i] = Math.signum(prePower[i]) * Range.clip( Math.abs(prePower[i]), om.cons.DRIVE_POWER_MINIMUM, om.cons.ROTATE_POWER_LIMIT);

        }

        setMotorPowerArray(setPower);

        while( (Math.abs(targetAngle - robotHeading) > om.cons.IMU_ROTATE_TOL) && (om.opModeIsActive() || om.testModeActive)) {
            //*************** ADDED CURRENT POSITION CALCULATION  FOR TELEMETRY ************************************

            currentPos = getMotorPosArray();
            //*************** ADDED CURRENT POSITION CALCULATION  FOR TELEMETRY ************************************


            angleUnWrap();
            om.updateIMU();

            for(int i = 0; i < 4; i++){

                prePower[i] = driveDirection[i] * (targetAngle - robotHeading) * om.cons.ROTATE_POWER_GAIN;

                setPower[i] = Math.signum(prePower[i]) * Range.clip( Math.abs(prePower[i]) , om.cons.DRIVE_POWER_MINIMUM, om.cons.ROTATE_POWER_LIMIT);

            }

            setMotorPowerArray(setPower);
            //*************** MODIFIED TELEMETRY FOR DEBUGGING ************************************

            om.telemetry.addData("Driving: ", step);
            om.telemetry.addData("Motor Counts: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
                    currentPos[0], currentPos[1], currentPos[2], currentPos[3]);
            om.telemetry.addData("Motor Power: ", "FL (%.2f) FR (%.2f) BR (%.2f) BL (%.2f)",
                    setPower[0],setPower[1],setPower[2],setPower[3]);
            om.telemetry.addData("Robot Heading: ", " Desired: %.2f, Actual: %.2f",targetAngle,robotHeading);
            om.telemetry.addData("Time: ", om.runtime);
            om.telemetry.addLine("=========================================");
            om.telemetry.update();
        }

        setMotorPower(0);

        currentPos = getMotorPosArray();
        angleUnWrap();
        om.updateIMU();

        om.telemetry.addData("COMPLETED Driving: ", step);
        om.telemetry.addData("Motor Counts: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
                currentPos[0], currentPos[1], currentPos[2], currentPos[3]);
        om.telemetry.addData("Robot Heading: ", " Desired: %.2f, Actual: %.2f",targetAngle,robotHeading);
        om.telemetry.addData("Time: ", om.runtime);
        om.telemetry.update();
        //*************** MODIFIED TELEMETRY FOR DEBUGGING ************************************

    }

//    public void driveRotateIMU(double angle, double powerLimit, String step, BasicAuto om) {
//
//        double error;
//        double steering;
//
//        // "angle" = the added rotation angle from the current position
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        angleUnWrap();
//        //Check tolerance zone to exit method
//        while (Math.abs(angle - robotHeading) > om.cons.IMU_ROTATE_TOL && ((om.opModeIsActive() || om.testModeActive))) {
//
//            error = angle - robotHeading;
//            steering = Range.clip((error * om.cons.GAIN), -om.cons.ROTATE_POWER_LIMIT, om.cons.ROTATE_POWER_LIMIT);
//
//            //update power limit
//            frontLeft.setPower(powerLimit - steering);
//            frontRight.setPower(powerLimit - steering);
//            backLeft.setPower(powerLimit - steering);
//            backRight.setPower(powerLimit - steering);
//
//            angleUnWrap();
//
//            om.telemetry.addData("Driving: %s", step);
//            om.telemetry.addData("Motor Counts: ", "FL (%d) FR (%d) BL (%d) BR (%d)",
//                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
//                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
//            om.telemetry.addData("wanted angle","%.2f, heading %.2f ", angle, robotHeading);
//            om.telemetry.update();
//
//            om.idle();
//        }
//
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backRight.setPower(0);
//        backLeft.setPower(0);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        om.telemetry.addLine("Power set to zero");
//        om.telemetry.update();
//
//    }


    public void drivePursuit(ArrayList<PursuitPoint> pathPoints, BasicAuto om, String step){
        double distanceToTarget = 100;
        double steeringPower;

        int[] driveDirection = new int[4];
        double[] setPower = new double[4];
        double prePower[] = new double[4];

        driveDirection[0] = -1;// FL
        driveDirection[1] = +1;// FR
        driveDirection[2] = +1;// BR
        driveDirection[3] = -1;// BL
        double maxLeft;
        double maxRight;
        double max;
        double steerThreshold = om.cons.STEERING_POWER_LIMIT * 0.25;
        double minFWDPower = om.cons.DRIVE_POWER_LIMIT*0.05;
        double ratio = (om.cons.DRIVE_POWER_LIMIT - minFWDPower) / (om.cons.STEERING_POWER_LIMIT - steerThreshold);

        boolean atEnd = false;
        double powerLimit = om.cons.DRIVE_POWER_LIMIT;
        double radius = om.cons.PURSUIT_RADIUS;

//        fieldX = robotFieldLocation.x;
//        fieldY = robotFieldLocation.y;

        robotNavigator(om);
        om.updateIMU();

        targetPoint = findPursuitPoint(pathPoints, robotFieldLocation, radius);

        int count = 0;
        boolean limitedPower = false;

        distanceToTarget = findDistance(new PursuitPoint(robotFieldLocation.x,robotFieldLocation.y),pathPoints.get(pathPoints.size()-1));

        while((distanceToTarget > radius) && (om.opModeIsActive() || om.testModeActive)) {
            angleUnWrap();//added for runnign on real hardware
            robotNavigator(om);
            om.updateIMU();

            targetPoint = findPursuitPoint(pathPoints, robotFieldLocation, radius);//robotLocation used is


            //navigator calculates its own angle, compare to IMU for accuracy?
            setTargetAngle(targetPoint,  robotFieldLocation);// calculates the angle to the target point - updates "targetHeading"
            steeringPower = calcSteeringPowerIMU(targetHeading,om);// "targetHeading" used for steering power calculation

            /**
             * needed ELSE to reset the powerLimit
             */
            if (Math.abs(steeringPower) > steerThreshold){
                powerLimit = om.cons.DRIVE_POWER_LIMIT - (ratio * (Math.abs(steeringPower) - steerThreshold));//added as new option (WAS 1.0 - )

                // need absolute value for direction of turns
                limitedPower = true;
            }
            else{//needed else for case that after steering is limited to reset
                powerLimit = om.cons.DRIVE_POWER_LIMIT;//need to reset powerLimit
                limitedPower = false;

            }


            for (int i = 0; i < 4; i++) {
                prePower[i] = (driveDirection[i] * powerLimit) + steeringPower;
            }

            maxLeft = Math.max(Math.abs(prePower[0]), Math.abs(prePower[3]));
            maxRight = Math.max(Math.abs(prePower[1]), Math.abs(prePower[2]));
            max = Math.max(maxLeft, maxRight);

            if (om.cons.DRIVE_POWER_LIMIT >= max) { max = 1;}//doesn't need "else" because max is already reclaculated above


            for (int i = 0; i < 4; i++) {
//                setPower[i] = Math.signum(prePower[i]) * Range.clip(Math.abs(prePower[i] / max), DRIVE_POWER_MINIMUM, DRIVE_POWER_LIMIT);// prior
                setPower[i] = Math.signum(prePower[i]) * Range.clip(Math.abs(prePower[i] / max), 0, om.cons.DRIVE_POWER_LIMIT);

            }
            setMotorPowerArray(setPower);
            distanceToTarget = findDistance(new PursuitPoint(robotFieldLocation.x,robotFieldLocation.y),pathPoints.get(pathPoints.size()-1));

            om.telemetry.addData("Driving", step);
            om.telemetry.addData("count", count);

//            om.telemetry.addData("Motor Commands: ", "FL (%d) FR (%d) BR (%d) BL (%d)",
//                    targetPos[0], targetPos[1],targetPos[2],targetPos[3]);
            om.telemetry.addData("Robot Heading", " Desired: %.2f, RobotNav: %.2f, FieldNav: %.2f, RobotHeading: %.2f", targetHeading, robotAngle, fieldAngle, robotHeading);
            om.telemetry.addData("Robot Location", " Desired(X,Y): (%.2f,%.2f), Navigator(X,Y): (%.2f,%.2f)",
                    targetPoint.x,targetPoint.y, robotFieldLocation.x, robotFieldLocation.y);

            om.telemetry.addData("Motor Counts", "FL (%d) FR (%d) BR (%d) BL (%d)",
                    flPrevious, frPrevious, brPrevious, blPrevious);
            om.telemetry.addData("Motor Power", "FL (%.2f) FR (%.2f) BR (%.2f) BL (%.2f)",
                    setPower[0], setPower[1], setPower[2], setPower[3]);
            om.telemetry.addData("Steering", "Power: %.3f, Gain: %.3f, LimitedPower: %s", steeringPower, om.cons.STEERING_POWER_GAIN,limitedPower);
            om.telemetry.addData("Drive Power", "powerLimit: %.3f, max: %.3f, DRIVE_POWER_LIMIT: %.2f,", powerLimit, max,om.cons.DRIVE_POWER_LIMIT);

            om.telemetry.addData("Distance to Target", "%.2f, target point (%.2f, %.2f)", distanceToTarget,pathPoints.get(pathPoints.size()-1).x,pathPoints.get(pathPoints.size()-1).y);
            om.telemetry.addLine("----------------------------------");
            om.telemetry.update();
            count += 1;

//            om.sleep(50);
        }
        setMotorPower(0.0);

    }
    /** METHODS FOR TELEOP DRIVING
     *
     */
    public void drivePower(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        forwardDirection = (-g1.left_stick_y * Math.pow(g1.left_stick_y, 2) ) * om.cons.TELEOP_DRIVE_POWER_LIMIT;
        rightDirection = (g1.right_stick_x * Math.pow(g1.right_stick_x, 2) ) * om.cons.TELEOP_DRIVE_POWER_LIMIT;

        frontLeft.setPower(Range.clip(-forwardDirection - rightDirection - clockwise, -om.cons.TELEOP_DRIVE_POWER_LIMIT, om.cons.TELEOP_DRIVE_POWER_LIMIT));
        frontRight.setPower(Range.clip(forwardDirection - rightDirection - clockwise, -om.cons.TELEOP_DRIVE_POWER_LIMIT, om.cons.TELEOP_DRIVE_POWER_LIMIT));
        backRight.setPower(Range.clip(forwardDirection + rightDirection - clockwise, -om.cons.TELEOP_DRIVE_POWER_LIMIT, om.cons.TELEOP_DRIVE_POWER_LIMIT));
        backLeft.setPower(Range.clip(-forwardDirection + rightDirection - clockwise, -om.cons.TELEOP_DRIVE_POWER_LIMIT, om.cons.TELEOP_DRIVE_POWER_LIMIT));
    }

    public void rotatePower(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        counterclockwiseDirection = (-g1.left_trigger * Math.pow(g1.left_trigger, 2) ) * om.cons.TELEOP_ROTATE_POWER_LIMIT;
        clockwiseDirection = (g1.right_trigger * Math.pow(g1.right_trigger, 2) ) * om.cons.TELEOP_ROTATE_POWER_LIMIT;

        clockwise = Range.clip(clockwiseDirection + counterclockwiseDirection, -om.cons.TELEOP_ROTATE_POWER_LIMIT, om.cons.TELEOP_ROTATE_POWER_LIMIT);

    }

    public void drivePowerAll(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        double powerLimit;

        if (g1.right_bumper)
            reduceDrivingPower = true;
        if (g1.left_bumper)
            reduceDrivingPower = false;

        if (reduceDrivingPower)
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT / 2;
        else
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT;

        forwardDirection = (-g1.left_stick_y) * powerLimit;
        rightDirection = (g1.left_stick_x) * powerLimit;
        clockwise = (g1.right_stick_x) * powerLimit;

        frontLeft.setPower(Range.clip(-forwardDirection - rightDirection - clockwise, -powerLimit, powerLimit));
        frontRight.setPower(Range.clip(forwardDirection - rightDirection - clockwise, -powerLimit, powerLimit));
        backRight.setPower(Range.clip(forwardDirection + rightDirection - clockwise, -powerLimit, powerLimit));
        backLeft.setPower(Range.clip(-forwardDirection + rightDirection - clockwise, -powerLimit, powerLimit));
    }

    public void drivePowerAllLeftStickScaled(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        double maxPower;
        double powerLimit;

        if (g1.right_bumper)
            reduceDrivingPower = true;
        if (g1.left_bumper)
            reduceDrivingPower = false;

        if (reduceDrivingPower)
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT / 2;
        else
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT;

        forwardDirection = (-g1.left_stick_y) * powerLimit;
        rightDirection = (g1.left_stick_x) * powerLimit;
        clockwise = (g1.right_stick_x) * powerLimit;

        maxPower = Math.abs(forwardDirection) + Math.abs(rightDirection) + Math.abs(clockwise);

        if (maxPower < om.cons.TELEOP_DRIVE_POWER_LIMIT) {
            maxPower = om.cons.TELEOP_DRIVE_POWER_LIMIT;
        }

        frontLeft.setPower((-(forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        frontRight.setPower(((forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        backRight.setPower(((forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        backLeft.setPower((-(forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);

    }

    public void drivePowerAllLeftStickScaledSquared(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        double maxPower;
        double powerLimit;

        if (g1.right_bumper)
            reduceDrivingPower = true;
        if (g1.left_bumper)
            reduceDrivingPower = false;

        if (reduceDrivingPower)
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT / 2;
        else
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT;

        forwardDirection = (-g1.left_stick_y * Math.abs(g1.left_stick_y)) * powerLimit;
        rightDirection = (g1.left_stick_x * Math.abs(g1.left_stick_x)) * powerLimit;
        clockwise = (g1.right_stick_x * Math.abs(g1.right_stick_x)) * powerLimit;

        maxPower = Math.abs(forwardDirection) + Math.abs(rightDirection) + Math.abs(clockwise);

        if (maxPower < om.cons.TELEOP_DRIVE_POWER_LIMIT) {
            maxPower = om.cons.TELEOP_DRIVE_POWER_LIMIT;
        }

        frontLeft.setPower((-(forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        frontRight.setPower(((forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        backRight.setPower(((forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);
        backLeft.setPower((-(forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * om.cons.TELEOP_DRIVE_POWER_LIMIT);

    }

    public void williamDrivePower(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        double maxPower;
        double x;
        double y;
        double absY;
        double absX;
        double powerLimit;

        x = g1.right_stick_x;
        y = g1.left_stick_y;
        absX = Math.abs(x);
        absY = Math.abs(y);

        if (g1.right_bumper)
            reduceDrivingPower = true;
        if (g1.left_bumper)
            reduceDrivingPower = false;

        if (reduceDrivingPower)
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT / 2;
        else
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT;

        counterclockwiseDirection = (-g1.left_trigger * g1.left_trigger) * powerLimit;
        clockwiseDirection = (g1.right_trigger * g1.right_trigger) * powerLimit;

        forwardDirection = ((-y * Math.pow(absY, 2) * (1 - absY)) + (-y * absY)) * powerLimit;
        rightDirection = ((x * Math.pow(absX, 2) * (1 - absX)) + (x * absX)) * powerLimit;

        clockwise = (clockwiseDirection + counterclockwiseDirection) * powerLimit;

        maxPower = Math.abs(forwardDirection) + Math.abs(rightDirection) + Math.abs(clockwise);

        if (maxPower < powerLimit) {
            maxPower = powerLimit;
        }

        frontLeft.setPower((-(forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        frontRight.setPower(((forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        backRight.setPower(((forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        backLeft.setPower((-(forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);

    }

    public void weightedDrivePower(Gamepad g1, Gamepad g2, BasicTeleOp om) {

        double maxPower;
        double x;
        double y;
        double RX;
        double absY;
        double absX;
        double absRX;
        double powerLimit;

        x = g1.left_stick_x;
        y = -g1.left_stick_y;
        RX = g1.right_stick_x;
        absX = Math.abs(x);
        absY = Math.abs(y);
        absRX = Math.abs(RX);

        if (g1.right_bumper)
            reduceDrivingPower = true;
        if (g1.left_bumper)
            reduceDrivingPower = false;

        if (reduceDrivingPower)
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT / 3;// was /2 one half
        else
            powerLimit = om.cons.TELEOP_DRIVE_POWER_LIMIT;

        forwardDirection = ((y * Math.pow(absY, 2) * (1 - absY)) + (y * absY)) * powerLimit;
        rightDirection = ((x * Math.pow(absX, 2) * (1 - absX)) + (x * absX)) * powerLimit;
//        clockwise = (g1.right_stick_x) * om.cons.TELEOP_ROTATE_POWER_LIMIT;
        clockwise = ((RX * Math.pow(absRX, 2) * (1 - absRX)) + (RX * absRX)) * powerLimit;

        maxPower = Math.abs(forwardDirection) + Math.abs(rightDirection) + Math.abs(clockwise);

        if (maxPower < powerLimit) {
            maxPower = powerLimit;
        }

        frontLeft.setPower((-(forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        frontRight.setPower(((forwardDirection/maxPower) - (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        backRight.setPower(((forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);
        backLeft.setPower((-(forwardDirection/maxPower) + (rightDirection/maxPower) - (clockwise/maxPower)) * powerLimit);

    }





}
