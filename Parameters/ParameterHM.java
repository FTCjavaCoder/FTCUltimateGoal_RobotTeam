package UltimateGoal_RobotTeam.Parameters;

import java.util.List;

public class ParameterHM {

    public enum instanceType {powerLimit, counts, toleranceCounts, distanceInches, rotationDegrees, servoPosition, controlGain}

    public boolean hasRange = true;
    public double min = -999;
    public double max = 999;
    public double increment = 0;
    public double value = 0;
    public String format = "%.2f";
    public instanceType paramType;
/* COACH NOTE: TO STREAMLINE THE EDITING OF PARAMETERS
 * Could assign "group" enum to each parameter that gets set when it's defined in CONSTANTS
 * THEN editHashMap() would ask which group to edit and reduce number of skips
 */
    public enum groupType {GENERAL, PURE_PURSUIT, SHOOTER_GAINS, TELEOP_LIMITS, AUTO_LIMITS, AUTO_ADJUSTMENTS}
    public groupType group = groupType.GENERAL;
    public ParameterHM(double inputValue, instanceType type, groupType gp) {// ADD groupType to CONSTRUCTOR
        group = gp;// ADD groupType to CONSTRUCTOR
        switch (type) {

            case powerLimit :
                value = inputValue;
                hasRange = true;
                min = -1;
                max = 1;
                increment = 0.01;
                paramType = type;
                break;

            case counts :
                value = inputValue;
                hasRange = false;
                increment = 100;
                format = "%.0f";
                paramType = type;
                break;

            case toleranceCounts :
                value = inputValue;
                hasRange = false;
                increment = 1;
                format = "%.0f";
                paramType = type;
                break;

            case distanceInches :
                value = inputValue;
                hasRange = false;
                increment = .5;
                paramType = type;
                break;

            case rotationDegrees :
                value = inputValue;
                hasRange = false;
                increment = .25;
                paramType = type;
                break;

            case servoPosition :
                value = inputValue;
                hasRange = true;
                min = 0;
                max = 1;
                increment = .1;
                paramType = type;
                break;

            case controlGain :
                value = inputValue;
                hasRange = true;
                min = 0;
                max = 1;
                increment = 0.0001;
                format = "%.5f";
                paramType = type;
                break;
        }

    }

    public void clipParameter() {

        if (value > max) {
            value = max;
        }
        if (value < min) {
            value = min;
        }
    }

    public void increaseParameter() {

        value += increment;
        clipParameter();
    }

    public void decreaseParameter() {

        value -= increment;
        clipParameter();
    }

    public void setParameter(double inputValue) {

        value = inputValue;
        clipParameter();
    }

    public int integerParameter() {
        int intValue;

     intValue = (int) Math.round(value);

     return intValue;
    }

    /* %%%%%% NEW COACH METHODS FOR SETTING 'groupType' & 'instanceType' from String */
    public static groupType setGroup(String groupName) {
        groupType gp = groupType.GENERAL;
        // below are the enum values for case, if enum is updated this method needs to update
        switch (groupName) {

            case ("GENERAL"):
                gp = groupType.GENERAL;
                break;
            case ("PURE_PURSUIT"):
                gp = groupType.PURE_PURSUIT;
                break;

            case ("SHOOTER_GAINS"):
                gp = groupType.SHOOTER_GAINS;
                break;

            case ("TELEOP_LIMITS"):
                gp = groupType.TELEOP_LIMITS;
                break;

            case ("AUTO_LIMITS"):
                gp = groupType.AUTO_LIMITS;
                break;

            case ("AUTO_ADJUSTMENTS"):
                gp = groupType.AUTO_ADJUSTMENTS;
                break;

        }
        return gp;
    }
    public static instanceType setInstance(String name) {
        instanceType inst = instanceType.counts;
        // below are the enum values for case, if enum is updated this method needs to update
        switch (name) {
            case ("powerLimit"):
                inst = instanceType.powerLimit;
                break;
            case ("counts"):
                inst = instanceType.counts;
                break;
            case ("toleranceCounts"):
                inst = instanceType.toleranceCounts;
                break;
            case ("distanceInches"):
                inst = instanceType.distanceInches;
                break;
            case ("rotationDegrees"):
                inst = instanceType.rotationDegrees;
                break;
            case ("servoPosition"):
                inst = instanceType.servoPosition;
                break;
            case ("controlGain"):
                inst = instanceType.controlGain;
                break;
        }
        return inst;
    }

}