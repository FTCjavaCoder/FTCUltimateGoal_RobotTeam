package UltimateGoal_RobotTeam.Parameters;

import android.content.Context;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.BufferedReader;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.HashMap;
import java.util.List;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;
import UltimateGoal_RobotTeam.Parameters.ParameterHM;
import UltimateGoal_RobotTeam.Parameters.ParameterHM.instanceType;// ADDED IMPORT TO REDUCE CONSTRUCTOR
import UltimateGoal_RobotTeam.Parameters.ParameterHM.groupType;// ADDED IMPORT TO REDUCE CONSTRUCTOR
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Constants {

    public HashMap<String, ParameterHM> pHM = new HashMap();
//    public HashMap<String, OptionAutonomous> aOHM = new HashMap();

    // Define the static power levels, power/speed limits, and initial positions for motors and servos
    public double DRIVE_POWER_LIMIT = 1.0;//chassis drive wheel (FR, FL, BR, BL) Motor power/speed limit
    public double ROTATE_POWER_LIMIT = 1.0;//clockwise rotation power/speed to be converted to individual motor powers/speeds
    public double DRIVE_POWER_MINIMUM = 0.1;
    public double STEERING_POWER_LIMIT = 0.6;
    public double STEERING_POWER_GAIN = 0.03;// NEED TO ASSESS WITH PURE PURSUIT
    public double POWER_GAIN = 0.2;
    public double ROTATE_POWER_GAIN = 0.02;
    public double IMU_ROTATE_TOL = 1.0;
    public double IMU_DISTANCE_TOL = 1.0;

    public double SHOOTER_POWER_LIMIT = 0.9;// was 1.0

    public int MOVE_TOL = 30;// tolerance for motor reaching final positions in drive methods

    public double TELEOP_DRIVE_POWER_LIMIT = 1.0;//chassis drive wheel (FR, FL, BR, BL) Motor power/speed limit for teleop
    public double TELEOP_ROTATE_POWER_LIMIT = 1.0;//chassis drive wheel (FR, FL, BR, BL) Motor power/speed limit for teleop

    //new Ultimate Goal variables
    public double PURSUIT_RADIUS = 6.0;

    // Shooter control parameters
    public double speedKI = 0.0005;
    public double maxError = 1000;
    public double speedKP = 0.0020;


    //Old Skystone variables
    public double doRotateMethod = 0;

    /* COACH NOTE: MOVE THE TRUE CONSTANTS TO THE HW CLASSES
     *  - ROBOT_INCH TO_MOTOR_DEG and DEG_TO_COUNTS and similar items belong in DRIVE TRAIN - don't need to be updated as parameters
     *  - DELETE OLD SKYSTONE VALUES
     */
    public final double ROBOT_INCH_TO_MOTOR_DEG = 360 / (3.875 * 3.14159); // units deg/inch - 360 deg. / wheel circumference (Wheel diameter x pi)
    public final int NUMBER_OF_JACK_STAGES = 3;// ASSUMING 3 PAIRS OF PIECES PER SIDE
    public final double MOTOR_DEG_TO_LEAD = 0.315/360; //Replace 0 with the distance between the ridges of the lead screw
    public final double SLIDE_INCH_TO_MOTOR_DEG = (3 / 3) * 360; // (3 screw rev / inch) * ( 1 motor rev / 3 screw rev) * 360 degrees / motor rev
    // 1/3 gear ratio 16 turns per inch screw
    // DERIVATION alpha = 2*AL/D; AL = arc length = wheel travel in inches, D = wheel diameter, alpha = wheel angle in radians
    // AL is input so conversion = 2/D * 180/pi (convert to degrees
    // alpha = AL * (360 / (D*pi))
    public double DEGREES_TO_COUNTS_40_1 = (1440.0/360.0) * (40.0/60.0); // units counts/degree - based on 1440 per 1 revolution
    public double DEGREES_TO_COUNTS_60_1 = 1440.0/360.0;

    public final double ROBOT_DEG_TO_WHEEL_INCH = 16.904807 * 3.14159 / 360;//NR wheel center to center 16.904807// units of inch/degree -- Robot rotation circumference [(wheel base (diagonal)] * pi/360 deg
    // DERIVATION AL = theta * RTD/2; AL = arc length = wheel travel in inches, RTD = robot turning diameter, theta = robot angle in radians
    // theta is input so conversion = RTD/2 * pi/180 (convert input in degrees to radians)
    // AL = theta * (RTD * pi/360)
    public double GAIN = 0.5;

    /*
     * Gripper and wrist disabled, by not calling the the properties
     */

    /* COACH NOTE: These are driveTrain constants and should belong there
     *
     */
    public final double adjForward = 0.964;// may simply be the difference between the two robots wheel diameters
    public final double adjRotate = 1.236;//NOT used because of using IMU to rotate to angle
    public final double adjRight = 1.079;//


//    public final double distanceFromStones = (48 - 16) - forwardFirstMove;

    /* COACH NOTE: these are Image Recognition constants and belong there (might already be there)
     *
     */
    public final double inchesPerPixel = 24 / 1280;// was 38 || inches per pixel for Tensor Flow to relate to vuforia zones
    public static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;// was BACK
    public static final String VUFORIA_KEY = " AUtTfjH/////AAAAGalBbST5Vkv8kqIoszZrsYOCBYcUVlFwYJ2rvrvVgm4ie/idQYx2x++SWi3UMEJnuP7Zww+cqOgyLepotRs9ppGxpCDcintIz0pIixMr+bievjJUDzdn0PyAJ6QUZK3xzoqDkTY1R99qvRORmTTqCx2/rGfYPlcOpdL5oWdhQsUatHyneF/eiWGBincPqBx3JfVwZnscF/B7J+u76YA4VTWLl8bl3iu26IYXMZE0zi7Pk+s9/pRQCCrLcxsvns4ouiSg3n61Z+jv8aS6y9ncwDdg0nm/XwDeiIrworkIPcPTW73LKpzX/63C1HprikXUJ+fm1eAkCfNy06n9SNTq20jxc7NXtGVUoE+WbNGmE4yb ";


    public Constants() {
        //Empty Constructor
    }

    public void defineParameters() {
        // ADD groupType to CONSTRUCTOR FOR ALL

        pHM.put("drivePowerLimit", new ParameterHM( 1.0, instanceType.powerLimit,groupType.PURE_PURSUIT));// was 0.75

        pHM.put("drivePowerMinimum", new ParameterHM( 0.07, instanceType.powerLimit,groupType.AUTO_LIMITS));// was 0.1

        pHM.put("rotatePowerLimit", new ParameterHM(1.0, instanceType.powerLimit,groupType.AUTO_LIMITS));// was 0.75

        pHM.put("shooterPowerLimit", new ParameterHM(0.9, instanceType.powerLimit,groupType.GENERAL));// was 1.0

        pHM.put("powerGain", new ParameterHM(0.1, instanceType.powerLimit,groupType.AUTO_ADJUSTMENTS));// was 0.2

        pHM.put("rotatePowerGain", new ParameterHM(0.01, instanceType.powerLimit, groupType.AUTO_ADJUSTMENTS));// was 0.02

        pHM.put("IMURotateTol", new ParameterHM(1.0, instanceType.rotationDegrees,groupType.AUTO_ADJUSTMENTS));// was 2.0

        pHM.put("IMUDistanceTol", new ParameterHM(1.0, instanceType.distanceInches,groupType.AUTO_ADJUSTMENTS));// new

        pHM.put("steeringPowerGain", new ParameterHM(0.03, instanceType.powerLimit,groupType.PURE_PURSUIT));// new

        pHM.put("steeringPowerLimit", new ParameterHM(0.6, instanceType.powerLimit,groupType.PURE_PURSUIT));// new

        pHM.put("teleOpDrivePowerLimit", new ParameterHM(1.0, instanceType.powerLimit,groupType.TELEOP_LIMITS));// was 0.55

        pHM.put("teleOpRotatePowerLimit", new ParameterHM(1.0, instanceType.powerLimit,groupType.TELEOP_LIMITS));// was 0.40

        pHM.put("pursuitRadius", new ParameterHM(6.0, instanceType.distanceInches,groupType.PURE_PURSUIT));

        pHM.put("speedKI", new ParameterHM(0.00025, instanceType.controlGain,groupType.SHOOTER_GAINS));

        pHM.put("maxError", new ParameterHM(1000, instanceType.counts,groupType.SHOOTER_GAINS));

        pHM.put("speedKP", new ParameterHM(0.0024, instanceType.controlGain,groupType.SHOOTER_GAINS));

    }// Define initial values for HashMap parameters

    public void initParameters() {

        for(String s:pHM.keySet()) {

            if(s.equals("drivePowerLimit")) {
                DRIVE_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("drivePowerMinimum")) {
                DRIVE_POWER_MINIMUM = pHM.get(s).value;
            }
            if(s.equals("rotatePowerLimit")) {
                ROTATE_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("shooterPowerLimit")) {
                SHOOTER_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("powerGain")) {
                POWER_GAIN = pHM.get(s).value;
            }
            if(s.equals("rotatePowerGain")) {
                ROTATE_POWER_GAIN = pHM.get(s).value;
            }
            if(s.equals("IMURotateTol")) {
                IMU_ROTATE_TOL = pHM.get(s).value;
            }
            if(s.equals("IMUDistanceTol")) {
                IMU_DISTANCE_TOL = pHM.get(s).value;
            }
            if(s.equals("steeringPowerLimit")) {
                STEERING_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("steeringPowerGain")) {
                STEERING_POWER_GAIN = pHM.get(s).value;
            }
            if(s.equals("teleOpDrivePowerLimit")) {
                TELEOP_DRIVE_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("teleOpRotatePowerLimit")) {
                TELEOP_ROTATE_POWER_LIMIT = pHM.get(s).value;
            }
            if(s.equals("pursuitRadius")) {
                PURSUIT_RADIUS = pHM.get(s).value;
            }
            if(s.equals("speedKI")) {
                speedKI = pHM.get(s).value;
            }
            if(s.equals("maxError")) {
                maxError = pHM.get(s).value;
            }
            if(s.equals("speedKP")) {
                speedKP = pHM.get(s).value;
            }

        }
    }

    public void writeToPhone(String fileName, BasicOpMode om) {
        Context c = om.hardwareMap.appContext;

        try {

            OutputStreamWriter osw = new OutputStreamWriter(c.openFileOutput(fileName, c.MODE_PRIVATE));

            for(String s : pHM.keySet()) {

                osw.write(s + "\n");
//                om.telemetry.addData("Parameter Name", "%s", s);
                osw.write(pHM.get(s).value + "\n");
//                om.telemetry.addData("Value", "%.2f", pHM.get(s).value);
                osw.write(pHM.get(s).paramType + "\n");
//                om.telemetry.addData("Type", "%s", pHM.get(s).paramType);
                osw.write(pHM.get(s).hasRange + "\n");
//                om.telemetry.addData("Range?", "%s", pHM.get(s).hasRange);
                osw.write(pHM.get(s).min + "\n");
//                om.telemetry.addData("Min", "%.2f", pHM.get(s).min);
                osw.write(pHM.get(s).max + "\n");
//                om.telemetry.addData("Max", "%.2f", pHM.get(s).max);
                osw.write(pHM.get(s).increment + "\n");
//                om.telemetry.addData("Increment", "%.2f", pHM.get(s).increment);
                osw.write(pHM.get(s).format + "\n");/* ADDED WRITING OUT format */
//                om.telemetry.update();
                osw.write(pHM.get(s).group + "\n");/* ADDED WRITING OUT group */

            }

            osw.close();
        }
        catch(Exception e) {

            Log.e("Exception", e.toString());

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
    }

    public boolean readFromPhone(String fileName, BasicOpMode om) {
        Context c = om.hardwareMap.appContext;
        boolean fileRead = false;//DEFAULT IS FALSE
        try {
            InputStream is = c.openFileInput(fileName);
            InputStreamReader isr = new InputStreamReader(is);
            BufferedReader br = new BufferedReader(isr);

            String s;
            while((s = br.readLine())!= null) {

                double v = Double.parseDouble(br.readLine());
                String t = br.readLine();
                /* READING IN ITEMS BELOW BUT NOT ASSIGNING
                 *  - ITEMS BELOW ARE WRITTEN OUT SO MUST BE READ IN TO MAINTAIN FILE FORMAT
                 *  - VALUES ARE IGNORED BECAUSE THEY ARE IN THE SWITCH CASES
                 *  - COULD SIMPLIFY CODE TO ONLY WRITE NAME, VALUE, TYPE in HASHMAP
                */
                String hr = br.readLine();
                double min = Double.parseDouble(br.readLine());
                double max = Double.parseDouble(br.readLine());
                double inc = Double.parseDouble(br.readLine());
                String form = br.readLine();/* READ IN FORMAT */
                String groupString= br.readLine();/* READ IN GROUP */

                /* NEW METHOD TO SET GROUP TYPE ADDED */
                groupType groupENUM = ParameterHM.setGroup(groupString);
                /* NEW METHOD TO SET INSTANCE TYPE ADDED */
                instanceType typeENUM = ParameterHM.setInstance(t);
                /* SIMPLIFIED CODE FOR READING IN VALUES & SETTING HashMap*/
                pHM.put(s, new ParameterHM(v, typeENUM,groupENUM));

                fileRead = true;/* CORRECTED TO RETURN THIS FROM METHOD vs. USE GLOBAL*/

                om.telemetry.addData("Parameter", "%s = "+ form, s, v);/* OUTPUT VALUE USING FORMAT */
//                om.telemetry.addData("Value", "%.2f", v);
//                om.telemetry.addData("Type", "%s", t);
//                om.telemetry.addData("Range?", "%s", hr);
//                om.telemetry.addData("Min", "%.2f", min);
//                om.telemetry.addData("Max", "%.2f", max);
//                om.telemetry.addData("Increment", "%.2f", inc);
                om.telemetry.addLine("---------------------------------");

                om.idle();
            }

            is.close();
        }
        catch(Exception e) {

            Log.e("Exception", e.toString());

            fileRead = false;

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
        return fileRead;
    }

    public void editHashMap(BasicOpMode om) {
        groupType[] gNames = groupType.values();
        int index = 0;
        boolean selectGroup = true;
        while(selectGroup && om.opModeIsActive()){//Allow looping to see multiple groups
            while(!om.gamepad1.y && om.opModeIsActive()) {
                om.telemetry.addLine("SELECT GROUP TO EDIT");
                // USE THE BUMPERS TO SET THE INDEX OF THE GROUP TO SELECT
                if(om.gamepad1.right_bumper) {
                    index += 1;
                    if(index > gNames.length-1){
                        index = gNames.length-1;
                    }
                    om.sleep(300);
                }
                if(om.gamepad1.left_bumper) {
                    index -= 1;
                    if(index < 0){
                        index = 0;
                    }
                    om.sleep(300);
                }
                // retrieve list of groups as a List
                for (groupType gp : gNames) {
                    om.telemetry.addData("\t", "%s", gp);
                }
                om.telemetry.addLine("USE BUMPERS TO CHANGE SELECTION");
                om.telemetry.addLine("\t RIGHT MOVES DOWN, LEFT MOVES UP");
                om.telemetry.addLine(" ");
                om.telemetry.addData("SELECTED GROUP:", "%s", gNames[index]);
                om.telemetry.addLine(" ");
                om.telemetry.addLine("PRESS 'Y' TO CONFIRM SELECTION");
                om.telemetry.update();
            }
            for(String s : pHM.keySet()) {
                if (pHM.get(s).group.equals(gNames[index])) {//ONLY RUN THROUGH THE ITEMS OF THE SELECTED GROUP
                    while (!(om.gamepad1.x || om.gamepad1.b) && om.opModeIsActive()) {
                        // X to EDIT || B to SKIP
                        om.telemetry.addData("Parameter Name", "%s", s);
                        om.telemetry.addData("Value", pHM.get(s).format, pHM.get(s).value);//USING FORMAT
                        om.telemetry.addData("Type", "%s", pHM.get(s).paramType);
                        om.telemetry.addData("Range?", "%s", pHM.get(s).hasRange);
                        om.telemetry.addData("Min", "%.2f", pHM.get(s).min);
                        om.telemetry.addData("Max", "%.2f", pHM.get(s).max);
                        om.telemetry.addData("Increment", pHM.get(s).format, pHM.get(s).increment);//USING FORMAT
                        om.telemetry.addLine("X to EDIT || B to SKIP");
                        om.telemetry.update();
                    }
                    if (om.gamepad1.x) {

                        while (!om.gamepad1.y && om.opModeIsActive()) {

                            om.telemetry.addData("Parameter Name", "%s", s);
                            om.telemetry.addData("Value", pHM.get(s).format, pHM.get(s).value);//USING FORMAT
                            om.telemetry.addData("Increment", pHM.get(s).format, pHM.get(s).increment);//USING FORMAT
                            om.telemetry.addLine("Right Bumper to increase, Left Bumper to decrease");
                            om.telemetry.addLine("Press Y to accept value");
                            om.telemetry.update();

                            if (om.gamepad1.right_bumper) {

                                pHM.get(s).increaseParameter();
                                om.sleep(300);
                            }
                            if (om.gamepad1.left_bumper) {

                                pHM.get(s).decreaseParameter();
                                om.sleep(300);
                            }
                        }
                    }
                    if (om.gamepad1.b) {

                        om.telemetry.addData("Skipped", "%s", s);
                        om.telemetry.update();
                        om.sleep(500);
                    }
                }

            }
            //DETERMINE WHETHER TO CONTINUE THROUGH MORE GROUPS
            while(!om.gamepad1.x && !om.gamepad1.b && om.opModeIsActive()) {
                om.telemetry.addLine("EDIT ANOTHER GROUP?");
                om.telemetry.addLine("X to CONTINUE || B  to END");
                om.telemetry.update();
                            }
            if (om.gamepad1.b) {
                selectGroup = false;
                om.telemetry.addLine("EDITING COMPLETE");
                om.telemetry.update();
                om.sleep(500);
            }
            else if (om.gamepad1.x) {
                selectGroup = true;
                om.telemetry.addLine("CONTINUING EDITING");
                om.telemetry.update();
                om.sleep(500);
            }

        }
    }

    public void writeToFile(String fileName, BasicOpMode om) {

        try {

            FileOutputStream fos = new FileOutputStream(fileName);
            OutputStreamWriter osw = new OutputStreamWriter(fos);

            for(String s : pHM.keySet()) {

                osw.write(s + "\n");
                om.telemetry.addData("Parameter Name", "%s", s);
                osw.write(pHM.get(s).value + "\n");
                om.telemetry.addData("Value", pHM.get(s).format, pHM.get(s).value);//USING FORMAT
                osw.write(pHM.get(s).paramType + "\n");
                om.telemetry.addData("Type", "%s", pHM.get(s).paramType);
                osw.write(pHM.get(s).hasRange + "\n");
                om.telemetry.addData("Range?", "%s", pHM.get(s).hasRange);
                osw.write(pHM.get(s).min + "\n");
                om.telemetry.addData("Min", "%.2f", pHM.get(s).min);
                osw.write(pHM.get(s).max + "\n");
                om.telemetry.addData("Max", "%.2f", pHM.get(s).max);
                osw.write(pHM.get(s).increment + "\n");
                om.telemetry.addData("Increment", pHM.get(s).format, pHM.get(s).increment);//USING FORMAT
                osw.write(pHM.get(s).format + "\n");//WRITE FORMAT
                om.telemetry.addData("Format", "%s", pHM.get(s).format);
                osw.write(pHM.get(s).group + "\n");/* ADDED WRITING OUT group */
                om.telemetry.addData("Group", "%s", pHM.get(s).group);
                om.telemetry.update();
            }

            osw.close();
        }
        catch(Exception e) {

//            Log.e("Exception", e.toString());

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
    }

    public boolean readFromFile(String fileName, BasicOpMode om) {
        boolean fileRead = false;//DEFAULT IS FALSE

        try {


            FileReader fr = new FileReader(fileName);
            BufferedReader br = new BufferedReader(fr);
            String s;
            while((s = br.readLine())!= null) {

                double v = Double.parseDouble(br.readLine());
                String t = br.readLine();
                /* READING IN ITEMS BELOW BUT NOT ASSIGNING
                 *  - ITEMS BELOW ARE WRITTEN OUT SO MUST BE READ IN TO MAINTAIN FILE FORMAT
                 *  - VALUES ARE IGNORED BECAUSE THEY ARE IN THE SWITCH CASES
                 *  - COULD SIMPLIFY CODE TO ONLY WRITE NAME, VALUE, TYPE in HASHMAP
                 */
                String hr = br.readLine();
                double min = Double.parseDouble(br.readLine());
                double max = Double.parseDouble(br.readLine());
                double inc = Double.parseDouble(br.readLine());
                String form = br.readLine();/* READ FORMAT */
                String groupString= br.readLine();/* READ IN GROUP */

                /* NEW METHOD TO SET GROUP TYPE ADDED */
                groupType groupENUM = ParameterHM.setGroup(groupString);
                /* NEW METHOD TO SET INSTANCE TYPE ADDED */
                instanceType typeENUM = ParameterHM.setInstance(t);
                /* SIMPLIFIED CODE FOR READING IN VALUES & SETTING HashMap*/
                pHM.put(s, new ParameterHM(v, typeENUM,groupENUM));

                fileRead = true;

                om.telemetry.addData("Parameter Name", "%s", s);
                om.telemetry.addData("Value", form, v);
                om.telemetry.addData("Type", "%s", t);
                om.telemetry.addData("Range?", "%s", hr);
                om.telemetry.addData("Min", "%.2f", min);
                om.telemetry.addData("Max", "%.2f", max);
                om.telemetry.addData("Increment", form, inc);
                om.telemetry.addData("Group", "%s", groupString);
                om.telemetry.addLine("/////////////////////////////");

                om.idle();
            }

            fr.close();
        }
        catch(Exception e) {

//            Log.e("Exception", e.toString());

            fileRead = false;

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
        return fileRead;
    }

    /* COACH NOTE: DELETE UNUSED ITEMS BELOW FROm SKYSTONE TEMPORARY CODE
     * THIS IS PRESERVED IN THE SKYSTONE REPOSITORY
     */

    // AUTONOMOUS OPTIONS
//    public void defineAutoOptions() {
//
//        aOHM.put("skyStoneInside", new OptionAutonomous(OptionAutonomous.name.skyStoneInside));
//
//        aOHM.put("skyStoneInsideUnmoved", new OptionAutonomous(OptionAutonomous.name.skyStoneInsideUnmoved));
//
//        aOHM.put("skyStoneOutside", new OptionAutonomous(OptionAutonomous.name.skyStoneOutside));
//
//        aOHM.put("skyStoneOutsideUnmoved", new OptionAutonomous(OptionAutonomous.name.skyStoneOutsideUnmoved));
//
//        aOHM.put("foundationInside", new OptionAutonomous(OptionAutonomous.name.foundationInside));
//
//        aOHM.put("foundationOutside", new OptionAutonomous(OptionAutonomous.name.foundationOutside));
//
//    }// Define Autonomous options
//
//    public void writeToPhoneAO(String fileName, BasicOpMode om) {
//        Context c = om.hardwareMap.appContext;
//
//        try {
//
//            OutputStreamWriter osw = new OutputStreamWriter(c.openFileOutput(fileName, c.MODE_PRIVATE));
//
//            for (String s : aOHM.keySet()) {
//
//                osw.write(s + "\n");
//                om.telemetry.addData("Option Name", "%s", s);
//                osw.write(aOHM.get(s).optionNumber + "\n");
//                om.telemetry.addData("Number", "%.2f", aOHM.get(s).optionNumber);
//                osw.write(aOHM.get(s).description + "\n");
//                om.telemetry.addData("Description", "%s", aOHM.get(s).description);
//
//                osw.close();
//            }
//        }
//        catch(Exception e){
//
//                Log.e("Exception", e.toString());
//
//                om.telemetry.addData("Exception", "%s", e.toString());
//                om.telemetry.update();
//            }
//
//    }
//
//    public void readFromPhoneAO(String fileName, BasicOpMode om) {
//        Context c = om.hardwareMap.appContext;
//
//        try {
//            InputStream is = c.openFileInput(fileName);
//            InputStreamReader isr = new InputStreamReader(is);
//            BufferedReader br = new BufferedReader(isr);
//
//            String s;
//            while((s = br.readLine())!= null) {
//
//                double n = Double.parseDouble(br.readLine());
//                String d = br.readLine();
//
//                switch (s) {
//
//                    case ("skyStoneOutside") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.skyStoneOutside));
//                        break;
//                    case ("skyStoneInside") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.skyStoneInside));
//                        break;
//
//                    case ("skyStoneOutsideUnmoved") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.skyStoneOutsideUnmoved));
//                        break;
//
//                    case ("skyStoneInsideUnmoved") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.skyStoneInsideUnmoved));
//                        break;
//
//                    case ("foundationOutside") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.foundationOutside));
//                        break;
//
//                    case ("foundationInside") :
//                        aOHM.put(s, new OptionAutonomous(OptionAutonomous.name.foundationInside));
//                        break;
//                }
//
//                om.fileWasRead = true;
//
//                om.telemetry.addData("Option Name", "%s", s);
//                om.telemetry.addData("Number", "%.2f", n);
//                om.telemetry.addData("Description", "%s", d);
//
//                om.idle();
//            }
//
//            is.close();
//        }
//        catch(Exception e) {
//
//            Log.e("Exception", e.toString());
//
//            om.fileWasRead = false;
//
//            om.telemetry.addData("Exception","%s", e.toString());
//            om.telemetry.update();
//        }
//    }
//
//    public void editHashMapAO(BasicOpMode om) {
//
//        for(String s : aOHM.keySet()) {
//
//            while(!(om.gamepad1.x || om.gamepad1.b) && om.opModeIsActive()) {
//                // X to EDIT || B to SKIP
//                om.telemetry.addData("Option Name", "%s", s);
//                om.telemetry.addData("Number", "%.2f", aOHM.get(s).optionNumber);
//                om.telemetry.addData("Description", "%s", aOHM.get(s).description);
//
//                om.telemetry.addLine("X to Select || B to SKIP");
//                om.telemetry.update();
//            }
//            if(om.gamepad1.x) {
//
//                while(!om.gamepad1.y && !om.gamepad1.b && om.opModeIsActive()) {
//
//                    om.telemetry.addData("Option Name", "%s", s);
//                    om.telemetry.addData("Number", "%.2f", aOHM.get(s).optionNumber);
//                    om.telemetry.addData("Description", "%s", aOHM.get(s).description);
//                    om.telemetry.addLine("Press Y to confirm or B to skip");
//                    om.telemetry.update();
//
//                    if(om.gamepad1.y) {
//// {skyStoneOutside, skyStoneInside, skyStoneOutsideUnmoved, skyStoneInsideUnmoved, foundationOutside, foundationInside}
//                        if(aOHM.get(s).optionNumber == 1) {
//
//                            om.selected = "skyStoneOutside";
//                        }
//                        if(aOHM.get(s).optionNumber == 2) {
//
//                            om.selected = "skyStoneInside";
//                        }
//                        if(aOHM.get(s).optionNumber == 3) {
//
//                            om.selected = "skyStoneOutsideUnmoved";
//                        }if(aOHM.get(s).optionNumber == 4) {
//
//                            om.selected = "skyStoneInsideUnmoved";
//                        }
//                        if(aOHM.get(s).optionNumber == 5) {
//
//                            om.selected = "foundationOutside";
//                        }
//                        if(aOHM.get(s).optionNumber == 6) {
//
//                            om.selected = "foundationInside";
//                        }
//
//                        om.sleep(300);
//                    }
//                }
//            }
//            if(om.gamepad1.b) {
//
//                om.telemetry.addData("Skipped","%s", s);
//                om.telemetry.update();
//                om.sleep(500);
//            }
//
//        }
//    }

}