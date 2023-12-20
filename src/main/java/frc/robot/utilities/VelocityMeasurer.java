package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

/**
 * <h3>VelocityMeasurementUtil</h3>
 * A class that acts as a tool for measuring velocity from two points in time.
 * <p>Useful for systems like the turret, where it is simpler to just calculate the velocity from measurements rather than try to interpret it from the encoder velocities.
 */
public class VelocityMeasurer {
    //The minimum amount of time that this system will consider when calculating velocity. Used to avoid dividing by zero.
    private static final double MIN_TIME = 0.001;

    private double previousMeasure;
    private double currentMeasure;
    private double previousTime;
    private double currentTime;

    public VelocityMeasurer(double startingPoint) {
        previousMeasure = startingPoint;
        currentMeasure = startingPoint;
        previousTime = Timer.getFPGATimestamp();
        currentTime = previousTime;
    }

    /**
     * Save the current measurement value of the system
     * @param value The most recent measurement of the system.
     * @param dt The amount of time elapsed between the most recent measurement and the one before that.
     * @return
     */
    public void saveVal(double value) {
        previousTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        previousMeasure = currentMeasure;
        currentMeasure = value;
    }
    
    public double getVelocity() {
        return (previousTime - currentTime) != 0 ? (currentMeasure - previousMeasure)/(previousTime - currentTime) : (currentMeasure - previousMeasure)/MIN_TIME;
    }
}
