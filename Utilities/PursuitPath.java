package UltimateGoal_RobotTeam.Utilities;

public class PursuitPath {

    /**
     * PursuitPath holds an array of points that define the driving path for the robot
     * The methods within the class are used to define basic 2D geometries and append them to the
     * arraylist of points.  The robot hardware class HardwareBilly implements drivePursuit as the
     * means for the robot to follow the points.
     *
     *  --- Are these methods really needed since not invoked? ----
     *  Recommend that the ArrayList fieldPoints is defined in BasicAuto - DONE!!
     *  Recommend that the method to findPursuitPoint is added to RobotHardware - DONE!!
     *  Recommend COMMENTING out this file and then DELETING after testing
     */
//
//    public ArrayList<PursuitLines> fieldLines = new ArrayList();
//    public ArrayList<PursuitPoint> fieldPoints = new ArrayList();
//
//    public ArrayList<PursuitLines> tempLocations = new ArrayList();
//    private final double TIME_STEP = 0.1;
//    private final int TOTAL_POINTS = (int) Math.round(30.0 / TIME_STEP);

    public PursuitPath() {
        //empty constructor to instantiate
    }

//    public void appendPoints() {
//        fieldLines.addAll(tempLocations);
//        tempLocations.clear();
//    }
//    public void defineLine(double x1, double y1,double x2, double y2){
//        PursuitLines L = new PursuitLines(x1, y1, x2, y2);
//        fieldPoints.add(new PursuitPoint(x1,y1));
//        fieldPoints.add(new PursuitPoint(x2,y2));
//
//        fieldLines.add(L);
//
//    }
//
//    public void defineRectangle(double x1, double y1, double w, double h) {
//        //Define rectangle from start x1, y1 to end at the same spot
//        //w and l can be negative or positive
////        double ratioWidth = Math.abs(w)/(2*(Math.abs(w)+Math.abs(h)));
////        double ratioHeight = Math.abs(h)/(2*(Math.abs(w)+Math.abs(h)));
//        PursuitLines L1;
//
//        //define first X segment
//        L1 = new PursuitLines(x1, y1, x1 + w, y1);
//        fieldLines.add(L1);
//
//        //define first Y segment
//        L1 = new PursuitLines(x1 + w, y1, x1 + w, y1 + h);
//        fieldLines.add(L1);
//
//        //define second X segment
//        L1 = new PursuitLines(x1 + w, y1 + h, x1, y1 + h);
//        fieldLines.add(L1);
//
//        //define second Y segment
//        L1 = new PursuitLines(x1, y1 + h, x1, y1);
//        fieldLines.add(L1);
//
//    }
//
//    public void defineArc(PursuitPoint center, double radius, double startAngleRad, double includedAngleRad, int points, pathDirection pd) {
//        //Follow a circle of radius, offset by startAngleRad radians, for a total angle of includedAngleRad,
//        //  divided into points
//        double maxPoints = points;
//        double scale = 1.0;
//        if(pd.equals(pathDirection.NEGATIVE)){
//            scale = -1.0;
//        }
//        for(int i =0; i<points;i++) {
//            double angle = scale * (i/maxPoints) * includedAngleRad;
//            double x = radius * Math.cos(angle - startAngleRad);
//            double y = radius * Math.sin(angle - startAngleRad);
//            fieldPoints.add(new PursuitPoint(x+center.x, y+center.y));
//        }
//
//    }
//
//    public enum pathDirection {POSITIVE,NEGATIVE};

}
