package UltimateGoal_RobotTeam.Utilities;

/** CLASS IS THE X, Y, and THETA LOCATION OF AN OBJECT ON THE FIELD
 * Contains additional data for delta distances, offsets, and whether objective is held
 * NEEDED FOR PURE PURSUIT & FIELD CONFIGURATION SO MOVED TO UTILITIES
 */
public class FieldLocation {
    public double x = 0;
    public double y = 0;
    public double deltaX = 0;
    public double deltaY = 0;
    public double theta = 0;
    public double thetaOffset = 0;
    public boolean heldByRobot = false;
    public boolean priorHold = false;
    public FieldLocation(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.thetaOffset = 0;
        this.heldByRobot = false;
        this.priorHold = false;

    }
    public void setHold(boolean held){
        this.priorHold = this.heldByRobot;
        this.heldByRobot = held;

    }
    public void setPriorHold(boolean ph){
        this.priorHold = ph;

    }
    public void setLocation(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

}