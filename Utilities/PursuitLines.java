package UltimateGoal_RobotTeam.Utilities;

public class PursuitLines {
    public double x1;
    public double y1;
    public double x2;
    public double y2;
    public double slope;
    public double b;

    /**
     * Relocate this class to a "Robot Classes" folder underneath the main Project
     * Collocate with other classes needed for actual robot code
     * Is the line class really needed or just the methods to offset and calcSlope?
     */

    public PursuitLines(double x1, double y1, double x2, double y2){
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        calcSlope();

    }
    public PursuitLines(PursuitPoint p1, PursuitPoint p2){
        this.x1 = p1.x;
        this.y1 = p1.y;
        this.x2 = p2.x;
        this.y2 = p2.y;
        calcSlope();

    }

    public void offset(double xo, double yo){
        this.x1 = x1-xo;
        this.y1 = y1-yo;
        this.x2 = x2-xo;
        this.y2 = y2-yo;
        calcSlope();
    }

    public void calcSlope(){
        this.slope = (y2 - y1) / (x2 - x1);
        this.b = y1 - this.slope*this.x1;

    }


}
