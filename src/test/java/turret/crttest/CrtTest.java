

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.CRTMath;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CrtTest {
    public static final double DELTA = 0.1;
    /* 
    @BeforeEach
    void setup() {}

    @AfterEach
    void shutdown() {}
    */

    /** 
     * Returns the error between the measured value given the two inputs and the expected value
     */
    double testOnce(double left, double right, double expected) {
        return expected - CRTMath.crt(left,right,TurretSubsystem.REDUCTION_LEFT,TurretSubsystem.REDUCTION_RIGHT);
    }

    @Test
    void testCrt() {
        double left = 6599.1111;
        double right = 5894.7725;
        double expected = 65;
        double received = CRTMath.crt(left,right,TurretSubsystem.REDUCTION_LEFT,TurretSubsystem.REDUCTION_RIGHT);
        System.out.println("Got: %s, Expected: %s, Error:%s".formatted(received,expected,received-expected)); 
        assertEquals(received,expected,DELTA);
    }
}