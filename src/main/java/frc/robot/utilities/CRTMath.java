package frc.robot.utilities;

public final class CRTMath {
    


    /**
     * <h3>Chinese Remainder Theorem</h3>
     * Finds the degrees of a center gear based on the ticks of two connected planetary gears.
     * <p> Currently hard-coded for the turret. </p>
     * @param left The ticks of the left encoder
     * @param right The ticks of the right encoder
     * @param leftReduction The gear ratio of the left encoder to the central gear.
     * @param rightReduction The gear ratio of the right encoder to the central gear.
     * @return The angle of the center gear.
     */
    public static double crt(double left, double right, double leftReduction, double rightReduction) {
        final int ticks = 8192;
        int lastPosition = -1;
        
        /*
            The way this logic works. The bigger reduction (5:1 > 4:1) will always find the solution last.
            So instead of building up an entire array of values for one ratio, and then crawling through 
            it as we build up the second array of values, we build them concurrently, but start with the
            smaller reduction. Then, we can find a solution sooner.
            
            This will always be faster than the alternative solution
        */
        double firstA = left;
        double lastA = right;
        double firstR = leftReduction;
        double lastR = rightReduction;
        
        if (leftReduction < rightReduction) {
            firstA = right;
            lastA = left;
            firstR = rightReduction;
            lastR = leftReduction;
        }
        
        for (int i = 0; i <= 10; i++) {
            int tickrement = i * ticks;
            int nextPosition = (int)((firstA+tickrement)*firstR+.5);
            double possibleSolution = (lastA+tickrement)*lastR;
            if (((int)(possibleSolution+.5)) == nextPosition || ((int)(possibleSolution+.5)) == lastPosition) {
                return possibleSolution*(360.0/ticks);
            }
            
            lastPosition = nextPosition;
        }
        return -1;
    }
}