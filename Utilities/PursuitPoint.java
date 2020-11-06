package UltimateGoal_RobotTeam.Utilities;

public class PursuitPoint {
    public double x;
    public double y;


    /**
     * Relocate this class to a "Robot Classes" folder underneath the main Project
     * Collocate with other classes needed for actual robot code
     */

    public PursuitPoint(double x, double y){
        this.x = x;
        this.y = y;

    }

    public void offset(double xo, double yo){
        this.x = x-xo;
        this.y = y-yo;

    }

    public void setPoint(double x, double y){
        this.x = x;
        this.y = y;

    }

}
