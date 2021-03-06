package UltimateGoal_RobotTeam.HarwareConfig;

//import OfflineCode.OfflineHW.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;

import UltimateGoal_RobotTeam.OpModes.BasicOpMode;

public class Shooter {

    private double shooter_Power = 0;// see getShooter_POwer() method to return
    /* COACH UPDATES: for shooter speed control
     * Added variables and constants below for control and trial & error calculations
     *  - Once working need to set constants as "final"
     *  - made methods to return private values for telemetry
     */
    //VARIABLES & CONSTANTS FOR SPEED CALCULATIONS
    private double userTargetRPM = 0.0;// this is the value updated by the user via the gamepad
    private int prevLeft = 0;//Shooter left motor previous position
    private int prevRight = 0;//Shooter right motor previous position
    private double speedLeft = 0.0;// Shooter left speed in units of counts/second
    private double speedRight = 0.0;//Shooter right speed in units of counts/second
    //VARIABLES & CONSTANTS FOR INTEGRAL CONTROL
    private double errorTotalLeft = 0.0;//Shooter left accumulated speed Error
    private double errorTotalRight = 0.0;//Shooter right accumulated speed Error
    private double powerKILeft = 0.0;//Shooter left Integral Gain Power
    private double powerKIRight = 0.0;//Shooter right Integral Gain Power
    private double SPEED_KI = 0.00025;//Integral gain power/RPM-s (based on sum of error), from initial trials 1/3/21
    // MAX_ERROR integral of error x SPEED_KI = 0.25 power
    // will never have > 0.25 power at 0.00025 gain, increasing gain with fixed max error wil result in fixed power
    // update for saturation of the error x KI and limit the integral
    private double MAX_ERROR = 1000;//Integral error limit ~= 1.0 / SPEED_KI = saturation limit to prevent integrator wind-up

    //VARIABLES & CONSTANTS FOR PROPORTIONAL CONTROL
    private double powerKPLeft = 0.0;//Shooter left Proportional Gain Power
    private double powerKPRight = 0.0;//Shooter right Proportional Gain Power
    private double SPEED_KP = 0.0024;// power/(RPM) proportional gain, from initial trials 1/3/21
    // 170 RPM error x SPEED_KP = 0.34 power
    // Note when it saturates

    //VARIABLES & CONSTANTS FOR OVERALL SPEED CONTROL
    private double prevTime = 0.0;// time from prior calculation
    private double deltaTime = 0.0;// time between calculations
    private double FF_GAIN = 0.000584;//Feed forward gain to respond to command, based on commanding power and calculating speed
    private double powerFeedForward = 0.0;//Shooter Feed Forward Power based on target speed not error
    private double powerLeft = 0.0;//Shooter left total power command (range -1.0 to 1.0)
    private double powerRight = 0.0;//Shooter right total power command (range -1.0 to 1.0)
    public boolean speedActive = false;//variable to toggle between speed control and power control for trial and error
    private final double COUNT_PER_REV = 103.6;// ANDYMARK NeveRest 3.7 gearmotor ticks(counts)/rev
    private final double MOTOR_MAX_SPEED = 1700.0;//motor speed capability in RPM, limits command
    /* Make below private final constants once done with troubleshooting */
    //UPDATE WITH LOOK-UP TABLE?
    // 103.6 counts per output revolution and 60 revs per minute
    // MAX SPEED = 1780 RPM (Andymark datasheet)
    // 1780 RPM = (1780 rev/min) * (103.6 count/rev) * (1 min/ 60s) = 3074 count/s
    // need to measure a whole count change so for 1 count in 1/3074 s = 0.0003 - need to check that dT > 10x 0.3 ms (3 ms) so that there will be a count measured for 10% speed
    // If updating every 20 - 50 ms then would measure a min speed of 1 count/.020 (1/.050) = 20 - 50 RPM
    // At max speed would measure 3074 counts/s * .020 (.050) = 61 - 153 counts
    // if off by 10% speed should saturate power 1.0 = ~ 175 RPM * (103.6 / 60) = 302.167 counts/s --> 1.0/302.167 = 0.0033
    //      In units of power/RPM = 1.0/175 =0.0057

    /* Public OpMode members. */
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;

    /**
     * CONSTRUCTORS
     * Taking the approach that when constructing robot all the initialization items can be run
     * Approach is that null Robot existing in BasicOpMode and then each OpMode constructs the Robot it needs
     * Methods within BasicAuto to be re-used would need to pass the Robot to have the correct robot config
     * This enables having similar code for different configs
     *
     * @param om: this is the OpMode that is constructing the
     * @param tm: this is the testMode boolean, if in testMode (Offline) then need to create new instances
     *            if NOT in testMode (real robot) need to map null objects
     *            Believe this can be made such that testMode robot is constructed on it's own and eliminates this boolean
     *            initTestMode is a separate constructor
     */

    public Shooter(BasicOpMode om, boolean tm) {
        if (tm) {
            om.telemetry.addData("Shooter", " Initializing ...");
            om.telemetry.update();

//            shooterLeft = new DcMotor();shooterRight = new DcMotor();shooterLeft.timeStep = om.timeStep;shooterRight.timeStep = om.timeStep;//NEEDED FOR OFFLINE


            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();

        } else {
            om.telemetry.addData("Shooter", " Initializing ...");
            om.telemetry.update();

            shooterLeft = om.hardwareMap.get(DcMotor.class, "motor_shooterL");
            shooterRight = om.hardwareMap.get(DcMotor.class, "motor_shooterR");

            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            /* COACH NOTE: CHANGED LEFT direction so all input can be the same sign */
            shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);//Spins CounterClockwise when viewed from above to shoot rings w/ positive power
            shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);//Spins Clockwise when viewed from above to shoot rings w/ positive power

            //Set 'constants' and gains here from parameters in OPMode
            SPEED_KI = om.cons.speedKI;
            MAX_ERROR = om.cons.maxError;
            SPEED_KP = om.cons.speedKP;

            //read in hashmap prior tp HW initialization
            om.telemetry.addLine("\t\t... Initialization COMPLETE");
            om.telemetry.update();
        }
    }

    /*
    --------------------------------------------------
                METHODS
    --------------------------------------------------
     */
    /*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                GAMEPAD METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     */
    public void setShooterPowerControl(Gamepad gamepad, BasicOpMode om) {
        // if speedActive and setShooterMode are removed then can remove the NOT(speedActive) outer loop
        // IF speedActive = TRUE THEN using external speed control and this method is inactive
        if (!speedActive) {
            if (gamepad.dpad_left) {
                shooter_Power = 0;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }

            if (gamepad.dpad_up) {
                shooter_Power += 0.05;//NOTE - direction changed for motor, dPad_up = increased power to Right and Left is now reversed
                shooterLeft.setPower(shooter_Power);//NOTE - direction changed for motor
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }
            if (gamepad.dpad_down) {
                shooter_Power -= 0.05;//NOTE - direction changed for motor, dPad_down = decreased power to Right and Left is now reversed
                shooterLeft.setPower(shooter_Power);//NOTE - direction changed for motor
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }
        }

    }

    public void setShooterPowerButton(Gamepad gamepad, BasicOpMode om) {
        // if speedActive and setShooterMode are removed then can remove the NOT(speedActive) outer loop
        // IF speedActive = TRUE THEN using external speed control and this method is inactive
        if (!speedActive) {
            if (gamepad.dpad_down) {
                shooter_Power = 0;
                shooterLeft.setPower(shooter_Power);
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }

            if (gamepad.dpad_left) {
                shooter_Power = 0.75;//NOTE - direction changed for motor, dPad_up = increased power to Right and Left is now reversed
                shooterLeft.setPower(shooter_Power);//NOTE - direction changed for motor
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }
            if (gamepad.dpad_up) {
                shooter_Power = 0.80;//NOTE - direction changed for motor, dPad_down = decreased power to Right and Left is now reversed
                shooterLeft.setPower(shooter_Power);//NOTE - direction changed for motor
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }
            if (gamepad.dpad_right) {
                shooter_Power = 0.85;//NOTE - direction changed for motor, dPad_down = decreased power to Right and Left is now reversed
                shooterLeft.setPower(shooter_Power);//NOTE - direction changed for motor
                shooterRight.setPower(shooter_Power);
                integratorReset();//reset integrator for new commands
                om.sleep(300);
            }
        }

    }

    public void setShooterSpeed(Gamepad gp, BasicOpMode om) {

        if (gp.dpad_left) {
            userTargetRPM = 0.0;//set speed to 0
            integratorReset();//reset integrator for new commands
            om.sleep(200);
        }
        if (gp.dpad_up) {
            userTargetRPM += 25.0;//increase target speed, was 100.0
            integratorReset();//reset integrator for new commands
            om.sleep(200);
        }
        if (gp.dpad_down) {
            userTargetRPM -= 25.0;//reduce target speed, was 100.0
            integratorReset();//reset integrator for new commands
            om.sleep(200);
        }
        userTargetRPM = Range.clip(userTargetRPM, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);//LIMIT output to be within motor capabilities
    }

    //REMOVE gain updates during run and make updates with hashmaps between runs
    public void setShooterKPGain(Gamepad gp, BasicOpMode om) {
        if (gp.left_stick_button) {
            SPEED_KP += 0.0002;//increase gain
            om.sleep(300);
        }
        if (gp.right_stick_button) {
            SPEED_KP -= 0.0002;//decrease gain
            om.sleep(300);
        }
        if (SPEED_KP < 0) {
            SPEED_KP = 0.0;//avoid negative gain and positive feedback
        }
    }

    public void setShooterKIGain(Gamepad gp, BasicOpMode om) {
        if (gp.left_bumper) {
            SPEED_KI -= 0.0001;//increase gain
            om.sleep(300);
        }
        if (gp.right_bumper) {
            SPEED_KI += 0.0001;//decrease gain
            om.sleep(300);
        }
        if (SPEED_KI < 0) {
            SPEED_KI = 0.0;//avoid negative gain and positive feedback
        }
    }

    public void setShooterModeGamePad(Gamepad gp, BasicOpMode om) {
        if (gp.dpad_right) {
            if (speedActive) {
                speedActive = false;
            } else {
                speedActive = true;
                integratorReset();
            }
            om.sleep(200);
        }
    }

    /* -- COACH NOTE:
     *
     *  - CREATED external speed control for Andymark motors since using RUN_WITH_ENCODERS not achieving full speed
     *    -- shooterSpeedControl() takes RPM and time since last measurement
     *    -- created supporting RPM and counts/s conversion methods convertRPM_to_CountPerSec & convertCountPerSec_to_RPM
     *    ** setShooterGain() and setShooterMode() are just for trial and error gain tuning - can be removed later
     *    - * setShooterSpeed() is the user control to go along with shooterSpeedControl() that can replace ShooterControl() once proven in
     */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       AUTONOMOUS OR MULTI-USE METHODS
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    public enum shooterSide {LEFT, RIGHT, AVERAGE};// enum used for get methods to select which motor or average

    /* -- COACH NOTE:
     *  - CREATED external speed control for Andymark motors since using RUN_WITH_ENCODERS not achieving full speed
     *    -- shooterSpeedControl() takes RPM and time since last measurement
     *    -- created supporting RPM and counts/s conversion methods convertRPM_to_CountPerSec & convertCountPerSec_to_RPM
     *
     */
    public void setShooterMode(boolean mode) {

        speedActive = mode;
    }
    /**
     * shooterSpeedControl() takes in a target speed and time increment to control the motor speeds
     * Calculations use FeedForward (FF_GAIN), Proportional (SPEED_KP), and Integral(SPEED_KI) Gains
     *
     * @param targetSpeed is a speed in RPM (Revolutions Per Minute)
     * @param om          is the BasicOpMode that is used to update the timing for the calculations
     *                    NOTE: there is a conversion method for counts/s to RPM and vice versa
     *                    - Currently this method is only active for setting the speed control when speedActive is TRUE
     *                    - this can be changed/removed once troubleshooting is over
     */
    public void shooterSpeedControl(double targetSpeed, BasicOpMode om) {
        // Input is target speed in RPM

        // Clip input to be within the motors capability
        targetSpeed = Range.clip(targetSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
        //CONSIDER UPDATING THIS TO HAVE BASIC CALCS PER MOTOR & REPEAT 2X, would control motors at different times
        deltaTime = calcDeltaTime(om);
        if (deltaTime > 0.003) { // if less elapsed time then too few counts and speed control will be inaccurate so skip
            // Get current motor positions
            int posLeft = shooterLeft.getCurrentPosition();
            int posRight = shooterRight.getCurrentPosition();

            //Find change in position from last time motors were sampled
            double deltaPosLeft = posLeft - prevLeft;
            double deltaPosRight = posRight - prevRight;

            // Calculate speed { speed = deltaPosition / deltaTime }
            //division of 2 doubles so should avoid integer division issues for counts
            speedLeft = convertCountPerSec_to_RPM(deltaPosLeft / deltaTime);// speed in units of RPM
            speedRight = convertCountPerSec_to_RPM(deltaPosRight / deltaTime);// speed in units of RPM

            //Calculate the power to apply to each motor based on known feed forward {expected power to achieve speed}
            powerFeedForward = FF_GAIN * targetSpeed; //feed forward power command

            //Calculate the error at this time step and total error
            double errorLeft = targetSpeed - speedLeft;
            double errorRight = targetSpeed - speedRight;
            errorTotalLeft += errorLeft * deltaTime;//integrate to allow more time to get to max error
            errorTotalRight += errorRight * deltaTime;//integrate to allow more time to get to max error
//            MAX_ERROR = 1.0/SPEED_KI;//REMOVE THIS AFTER TRIAL AND ERROR - MAX_ERROR & SPEED_KI should be final
            errorTotalLeft = Range.clip(errorTotalLeft, -MAX_ERROR, MAX_ERROR);//limit total error to avoid integrator power > 1
            errorTotalRight = Range.clip(errorTotalRight, -MAX_ERROR, MAX_ERROR);

            //Calculate the power to apply to each motor based on error { error = targetSpeed - calculatedSpeed)
            //Use Range.clip to ensure calculated power is within motor control range of -1.0 to 1.0
            //Proportional gain power based on current target vs. measurement
            powerKPLeft = Range.clip(SPEED_KP * errorLeft, -1.0, 1.0);// power = gain x error (target - calculated)
            powerKPRight = Range.clip(SPEED_KP * errorRight, -1.0, 1.0);//Right and left turn opposite directions but motors are configured to be opposite
            //Integral (sum or error) gain power based on current total error
            powerKILeft = Range.clip(SPEED_KI * errorTotalLeft, -1.0, 1.0);// power = gain x error (target - calculated)
            powerKIRight = Range.clip(SPEED_KI * errorTotalRight, -1.0, 1.0);//Right and left turn opposite directions but motors are configured to be opposite

            //Combine FF, KP, and and KI powers
            powerLeft = Range.clip(powerFeedForward + powerKPLeft + powerKILeft, -1.0, 1.0);//clip to ensure within -1.0 to 1.0
            powerRight = Range.clip(powerFeedForward + powerKPRight + powerKIRight, -1.0, 1.0);//clip to ensure within -1.0 to 1.0

            // if speedActive and setShooterMode are removed then can remove the (speedActive) if statement
            // IF speedActive = FALSE THEN using power control and this method is inactive
            if (speedActive) {
                //Set motor powers
                shooterLeft.setPower(powerLeft);
                shooterRight.setPower(powerRight);
                shooter_Power = (powerLeft + powerRight) / 2.0;//keep shooter_power updated as average of both motors
            }
            //Update the prior positions to the values used here
            prevLeft = posLeft;
            prevRight = posRight;


        }
    }
    //ADD gamepad control to reset integrator - hold in reset until speedActive = TRUE?

    public void integratorReset() {
        // Reset integral power input and total errors
        // may be needed with hand-over of control
        errorTotalLeft = 0.0;
        errorTotalRight = 0.0;
        powerKILeft = 0.0;
        powerKIRight = 0.0;
    }

    /* %%%%%%%  TELEMETRY  %%%%% */
    public void getMaxTelemetry(BasicOpMode om) {
        om.telemetry.addData("SHOOTER SPEED MODE"," %s",speedActive);
        om.telemetry.addData("\tKP GAIN","  %.5f",SPEED_KP);
        om.telemetry.addData("\tKI GAIN","  %.5f",SPEED_KI);
        om.telemetry.addData("\tFF GAIN","  %.5f",FF_GAIN);
        om.telemetry.addData("\tdt","  %.4f seconds",deltaTime);//check deltaTime and see if slower with more method calls
        om.telemetry.addLine("-----------------------------------------------------------------------\n");
        om.telemetry.addLine("POSITION:");
        om.telemetry.addData("\tCurrent Cnt","L: %d, R: %d",shooterLeft.getCurrentPosition(), shooterRight.getCurrentPosition());//returns doubles
        om.telemetry.addData("\tPrev Counts","L: %d, R: %d",prevLeft,prevRight);

        om.telemetry.addLine("SPEED:");
        om.telemetry.addData("\tUser Target(RPM)","     %.1f,",userTargetRPM);
        om.telemetry.addData("\tCalculated(RPM)","Avg:  %.1f, L = %.1f, R = %.1f",(speedRight + speedLeft)/2.0,speedLeft,speedRight);

        om.telemetry.addLine("POWER:");
        om.telemetry.addData("\tFeed Forward","%.3f",powerFeedForward);
        om.telemetry.addData("\tFeedback P","L: %.3f, R: %.3f",powerKPLeft,powerKPRight);
        om.telemetry.addData("\tFeedback I","L: %.3f, R: %.3f",powerKILeft,powerKIRight);
        om.telemetry.addData("\tTotal","    L: %.3f, R: %.3f",powerLeft,powerRight);
        om.telemetry.addData("\tActual","   L: %.2f, R: %.2f",shooterLeft.getPower(),shooterRight.getPower());
        // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
        //Write to RobotLog methods are static so always accessible?
        //LOG HEADERS"\tTime(s)\tDelta Time (s)\tSpeedActive\tTarget Speed (RPM)\tLeft Speed (RPM)\tRight Speed (RPM)\tLeft Power\tRight Power");

        RobotLog.ii("Shooter Speed", "\t%.3f\t%.3f\t%s\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f", om.runtime.time(),
                deltaTime,speedActive,userTargetRPM,speedLeft,speedRight,shooterLeft.getPower(),shooterRight.getPower());

    }
    public void getBasicTelemetry(BasicOpMode om) {
        om.telemetry.addData("SHOOTER SPEED MODE"," %s",speedActive);
        om.telemetry.addLine("SPEED:");
        om.telemetry.addData("\tUser Target(RPM)","     %.1f,",userTargetRPM);
        om.telemetry.addData("\tCalculated(RPM)","Avg:  %.1f, L = %.1f, R = %.1f",(speedRight + speedLeft)/2.0,speedLeft,speedRight);
        om.telemetry.addLine("POWER:");
        om.telemetry.addData("\tVariable", "Shooter_Power %.2f", getShooter_Power());
        om.telemetry.addData("\tMotor Power", " L = %.2f, R = %.2f",shooterLeft.getPower(), shooterRight.getPower());
        // NO telemetry.update() since more info will be added at RobotHWMulti and/or OpMode level
    }


    public void shutdown(){
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
    }

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                BASIC SET & GET METHODS
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    public void setShooter_Power(double power){
        shooter_Power = power;
        shooterLeft.setPower(shooter_Power);
        shooterRight.setPower(shooter_Power);//NOTE - direction changed for motor
    }
    public double getShooter_Power(){
        return shooter_Power;
    }
    public boolean getSpeedMode(){
        return speedActive;
    }
    public double getUserTargetSpeed(){
        return userTargetRPM;
    }
    public void setUserTargetSpeed(double speed){
        userTargetRPM = speed;
    }
    public double getDT(){
        return deltaTime;
    }
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                COMPLEX SET & GET METHODS
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /** getSpeedArray() used to provide the internal speed values out to another class
     * @return speedArray[] is length 4
     * speedArray[0] = left;
     * speedArray[1] = right;
     * speedArray[2] = average;
     * speedArray[3] = target;
     */
    public double[] getSpeedArray(){
        double[] speedArray = new double[4];
        speedArray[0] = speedLeft;
        speedArray[1] = speedRight;
        speedArray[2] = (speedLeft + speedRight)/2.0;
        speedArray[3] = userTargetRPM;

        return speedArray;
    }

    /** getPowerArray() used to provide the internal power values out to another class
     * @param side is an enum of type shooterSide with values of {LEFT,RIGHT,AVERAGE}
     *             side will set the type of data returned for a specific motor or the average
     * @return powerArray[] is length 5
     * powerArray[0] = FF;
     * powerArray[1] = KP;
     * powerArray[2] = KI;
     * powerArray[3] = Total;
     * powerArray[4] = reported from motor;
     */
    public double[] getPowerArray(shooterSide side){
        double[] powerArray = new double[5];
        powerArray[0] = powerFeedForward;
        switch(side) {
            case LEFT:
                powerArray[1] = powerKPLeft;
                powerArray[2] = powerKILeft;
                powerArray[3] = powerLeft;
                powerArray[4] = shooterLeft.getPower();
                break;
            case RIGHT:
                powerArray[1] = powerKPRight;
                powerArray[2] = powerKIRight;
                powerArray[3] = powerRight;
                powerArray[4] = shooterRight.getPower();
                break;
            case AVERAGE:
                powerArray[1] = (powerKPLeft + powerKPRight)/2.0;
                powerArray[2] = (powerKILeft + powerKIRight)/2.0;
                powerArray[3] = (powerLeft + powerRight)/2.0;
                powerArray[4] = (shooterLeft.getPower() + shooterRight.getPower())/2.0;
                break;
        }
        return powerArray;
    }

    /** getPosArray() used to provide the internal position values out to another class
     * NOTE: this return type DOUBLE vs. INT since there's an average
     * @param side is an enum of type shooterSide with values of {LEFT,RIGHT,AVERAGE}
     *             side will set the type of data returned for a specific motor or the average
     * @return posArray[] is length 3
     * posArray[0] = current;
     * posArray[1] = previous;
     * posArray[2] = target;
     */
    public double[] getPosArray(shooterSide side){
        double[] posArray = new double[3];
        switch(side) {
            case LEFT:
                posArray[0] = shooterLeft.getCurrentPosition();
                posArray[1] = prevLeft;
                posArray[2] = shooterLeft.getTargetPosition();
                break;
            case RIGHT:
                posArray[0] = shooterRight.getCurrentPosition();
                posArray[1] = prevRight;
                posArray[2] = shooterRight.getTargetPosition();
                break;
            case AVERAGE:
                posArray[0] = (shooterLeft.getCurrentPosition() + shooterRight.getCurrentPosition())/2.0;
                posArray[1] = (prevLeft + prevRight)/2.0;
                posArray[2] = (shooterLeft.getTargetPosition() + shooterRight.getTargetPosition())/2.0;
                break;
        }
        return posArray;
    }

    /** gainArray() used to provide the internal speed control gain values out to another class
     * @return speedArray[] is length 3
     * gainArray[0] = FF;
     * gainArray[1] = KP;
     * gainArray[2] = KI;
     */
    public double[] getGainArray(){
        double[] gainArray = new double[3];
        gainArray[0] = FF_GAIN;
        gainArray[1] = SPEED_KP;
        gainArray[2] = SPEED_KI;
        return gainArray;
    }

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     CONVERSION & INTERMEDIATE CALCULATION METHODS
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    public double convertRPM_to_CountPerSec(double RPM){
        return RPM * COUNT_PER_REV/ 60.0;
    }
    public double convertCountPerSec_to_RPM(double countsPerSec){
        return countsPerSec * 60.0 / COUNT_PER_REV;
    }
    public double calcDeltaTime(BasicOpMode om){
        double tEnd = om.runtime.time();
        double deltaTime = tEnd - prevTime;
        prevTime = tEnd;
        return deltaTime;
    }


}
