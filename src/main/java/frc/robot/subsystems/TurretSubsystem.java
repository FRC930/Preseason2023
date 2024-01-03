package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CRTMath;
import frc.robot.utilities.VelocityMeasurer;

public class TurretSubsystem extends SubsystemBase implements AutoCloseable{
    //#region Constants
    public static final double REDUCTION_LEFT = 0.2;
    public static final double REDUCTION_RIGHT = 0.21; 
    //#endregion

    //#region Hardware
    private final CANSparkMax motorL;
    private final CANSparkMax motorR;

    private final AbsoluteEncoder encoderL;
    private final AbsoluteEncoder encoderR;
    
    //#endregion

    //I store the position to avoid having to call getPosition twice per tick. 
    //I was told we had to call it once per tick already, so we might as well move it to the periodic to get more accurate velocity measurements.
    private double currentPosition;
    private final VelocityMeasurer velocityMeasurer;
    
    public TurretSubsystem(
        int motorLId,
        int motorRId
    ) {
        this.motorL = new CANSparkMax(motorLId, MotorType.kBrushless);
        this.motorR = new CANSparkMax(motorRId, MotorType.kBrushless);
        this.encoderL = motorL.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.encoderR = motorR.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        currentPosition = getRawPosition();
        velocityMeasurer = new VelocityMeasurer(currentPosition);
    }

    /**
     * <h3>getPosition</h3>
     * Gets the position of the turret using Chinese Remainder Theorem.
     * @return The position in degrees
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * <h3>getRawPosition</h3>
     * Gets the position of the turret using Chinese Remainder Theorem.
     * <p>Freshly caluclates the position rather than using the value cached by periodic().
     * <p>Only use if you know EXACTLY what you're doing. Use getPosition() otherwise
     * @return The position in degrees
     */
    public double getRawPosition() {
        return CRTMath.crt(encoderL.getPosition(),encoderR.getPosition(),REDUCTION_LEFT,REDUCTION_RIGHT);
    }

    
    /**
     * <h3>getVelocity</h3>
     * Gets the current turn rate of the turret 
     * @return the turn rate of the turret in degrees/second. Positive values for clockwise, negative values for counterclockwise.
     */
    public double getVelocity() {
        return velocityMeasurer.getVelocity();
    }

    /**
     * <h3>setSpeed</h3>
     * Sets the desired motor percent output to the turret.
     * @param speed The desired speed of the motors in percent output. Positive values turn clockwise, negative turn counterclockwise. 
     * 
     */
    public void setSpeed(double speed) {
        // Clamps [-1,1]
        double setPoint = speed < -1 ? -1 : (speed > 1 ? 1 : speed);
        motorL.set(setPoint);
        motorR.set(setPoint);
    }

    @Override
    public void periodic() {
        currentPosition = CRTMath.crt(encoderL.getPosition(),encoderR.getPosition(),REDUCTION_LEFT,REDUCTION_RIGHT);
        velocityMeasurer.saveVal(currentPosition);
    }

    @Override
    public void close() throws Exception {
        motorL.close();
        motorR.close();
    }
}
