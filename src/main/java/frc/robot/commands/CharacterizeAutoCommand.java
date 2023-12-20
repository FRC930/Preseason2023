 package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class CharacterizeAutoCommand extends Command {

    private static final double NUMBER_OF_FEET_BEFORE_STOP = 10.0;
    private static final double DEFAULT_SLOW_QUASISTIC_STEP = 0.25;
    private static final double DEFAULT_FAST_DYNAMIC_VOLTS = 6.0;

    private final double m_slowQuasisticStep;
    private final double m_fastDynamicVolts;

    private static final double MS_20 = .02; 

    // TODO USE VALUES (rotations)
    private static final int RADIUS = 2; // Inches
    private static final double TICKS_PER_ROTATION = 2048.0;
    private static final double GEAR_RATIO = 8.16; // 6.86 

    //"Meters", "Feet", "Inches", "Radians", "Rotations", and "Degrees".
    private static final String UNITS = "Rotations"; 
    private static final double UNITS_PER_ROTATIONS = 1.0; 
    // private static final String UNITS = "Inches"; 
    // private static final double UNITS_PER_ROTATIONS = RADIUS*2*Math.PI; // TODO detmine unit per 

    private String m_jsonObjectName;
    private double m_directionMultiplier=1.0;
    private final double m_quasistaticStep; 
    private JSONArray m_frontTimeEntries = null;
    private JSONArray m_backTimeEntries = null;
    private SwerveDrivetrain m_drivetrain;
    private SwerveModule m_modFL;
    private SwerveModule m_modFR;
    private SwerveModule m_modBL;
    private SwerveModule m_modBR;
    private double m_quasistaticVolts = 0.0;
    private double m_startMeters;

    private DataLog log = DataLogManager.getLog();
    private double m_lastVolts = -1.0;
    private DoubleLogEntry voltsLog = new DoubleLogEntry(log,this.getClass().getSimpleName()+"/Volts");
    
/*
 * Command structure
 * https://github.com/r4stered/RobotMake/blob/main/robotProgram/src/subsystems/DrivebaseSubsystem.cpp
 * DATA FORMAT (DRIVE TRAIN)
 * https://github.com/wpilibsuite/allwpilib/blob/main/sysid/docs/data-collection.md
 */
   
    /**
     * CharacterizeAutoCommand
     * <pre>
     * - Look for // TODO CHARACTERIZATION
     * -- DONT SET RobotContainer.java defaultcommand for drivetrain (it causes voltage to jitter??? but defaultcommand should be disabled during run)
     * -- DONT use TunerConstants.java .withDriveMotorGains()   (but leave steer gains)
     * - align wheels with belves turned to center of robot
     * - use auto selection on shuffleboard (slow-forward/slow-back/fast-forward/fast-back/write-file) 
     * -- enable/disable driver station each auto
     * - get files from roborio lvuser/lvuser sysid_data_*.json  (lvuser/lvuser or admin)
     * -- https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ftp.html
     * - Use Sysid to analyze log files .withDriveMotorGains()
     * -- use either each log file sysid_data_*.json (front wheels)/sysid_data_back_*.json(back wheels)
     * -- Update TunerConstants.java driveGains  PID/ksKvKa
     * 
     * TODO:
     * - how to get wheel in correct position on startup
     * - how to get simulation consistantly working with it (was defaultcommand on drivetrain -- had odometry thread)
     * - how to get characterization code left enabled in robot code.
     * - able to set each module with driveGains (characterize each module)
     * </pre>
     * @param drivetrain
     * @param forward
     * @param fastSpeed
     * @param frontTimeEntries
     * @param backTimeEntries
     */
    public CharacterizeAutoCommand(SwerveDrivetrainSubsystem drivetrain, 
        boolean forward,
        boolean fastSpeed,
        JSONArray frontTimeEntries,
        JSONArray backTimeEntries,
        Double quasistaticStepForSlow,
        Double dynamicVoltsForFast)
    {
        if(quasistaticStepForSlow != null && quasistaticStepForSlow>0.0 && quasistaticStepForSlow < 1.0) {
            m_slowQuasisticStep = quasistaticStepForSlow;
        } else {
            m_slowQuasisticStep = DEFAULT_SLOW_QUASISTIC_STEP;
        }
        if(dynamicVoltsForFast != null && dynamicVoltsForFast>0.0 && dynamicVoltsForFast < 12.0) {
            m_fastDynamicVolts = dynamicVoltsForFast;
        } else {
            m_fastDynamicVolts = DEFAULT_FAST_DYNAMIC_VOLTS;
        }
        m_frontTimeEntries = frontTimeEntries;
        m_backTimeEntries = backTimeEntries;
        m_drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        String stageSpeed;
        if(fastSpeed) {
            m_quasistaticStep = m_fastDynamicVolts;
            stageSpeed="fast";
        } else {
            m_quasistaticStep = m_slowQuasisticStep;
            stageSpeed="slow";
        }
        String stageDirection;
        if(forward) {
            m_directionMultiplier = 1.0;
            stageDirection="forward";
        } else {
            m_directionMultiplier = -1.0;
            stageDirection="backward";
        }
        m_jsonObjectName = stageSpeed+"-"+stageDirection;

        m_modFL = m_drivetrain.getModule(0);
        m_modFR = m_drivetrain.getModule(1);
        m_modBL = m_drivetrain.getModule(2);
        m_modBR = m_drivetrain.getModule(3);
    }

    private void setDriveVoltage(double v) {
        m_modFL.getDriveMotor().setVoltage(v);
        m_modFR.getDriveMotor().setVoltage(v);
        m_modBL.getDriveMotor().setVoltage(v);
        m_modBR.getDriveMotor().setVoltage(v);

        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"Volts", v);
        // TRY out data logger
        if(m_lastVolts != v) {
            voltsLog.append(v);
            m_lastVolts = v;
        }
    }

    private double convertRotationToUnits(double rotations) {
        // TODO is this how to convert? WHAT ABOUT TICKS_PER_ROTATION
        // return rotations/(GEAR_RATIO*UNITS_PER_ROTATIONS);
        //return rotations*UNITS_PER_ROTATIONS/GEAR_RATIO;
        return rotations;
    }

    private void trackModuleData(JSONArray data, SwerveModule leftModule, SwerveModule rightModule) {
        // log each time
        //  timestamp 1,
        data.add(Timer.getFPGATimestamp()); // TODO DOUBLE?
        //  l voltage 1,
        data.add(leftModule.getDriveMotor().getMotorVoltage().getValue()); 
        //  r voltage 1,
        data.add(rightModule.getDriveMotor().getMotorVoltage().getValue()); 
        //  l position 1,
        data.add(convertRotationToUnits(leftModule.getDriveMotor().getPosition().getValue())); 
        //   r position 1,
        data.add(convertRotationToUnits(rightModule.getDriveMotor().getPosition().getValue())); 
        //  l velocity 1,
        data.add(convertRotationToUnits(leftModule.getDriveMotor().getVelocity().getValue())); 
        //  r velocity 1,
        data.add(convertRotationToUnits(rightModule.getDriveMotor().getVelocity().getValue())); 
        //  angle 1,
        data.add(0.0); 
        // angular rate 1
        data.add(0.0);
    }

    
    static public void initializeDirection(SwerveDrivetrainSubsystem driveTrain) {
        // driveTrain.tareEverything();
    }
    
    public void initializeDirectionNonStatic() {
        // Set wheels in inital postion
        m_modFL.getSteerMotor().setControl(new MotionMagicVoltage(0));
        m_modFR.getSteerMotor().setControl(new MotionMagicVoltage(0));
        m_modBL.getSteerMotor().setControl(new MotionMagicVoltage(0));
        m_modBR.getSteerMotor().setControl(new MotionMagicVoltage(0));
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning(m_jsonObjectName+" Starting...", false);
        m_quasistaticVolts=0.0;
        setDriveVoltage(m_quasistaticVolts);

        // TODO TRY TO GET STEER wheel to be in initial position!!!!
        initializeDirectionNonStatic();
    
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"START_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"END_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());

        // TODO reset everything  (can not seem to run multiple times)
        m_frontTimeEntries.clear();
        m_backTimeEntries.clear();
        m_startMeters = (m_modFL.getCachedPosition().distanceMeters);
    }

    @Override
    public void execute() {
        // Increase power to drive
        if(m_quasistaticStep != m_fastDynamicVolts) {
            m_quasistaticVolts = m_quasistaticVolts + (m_directionMultiplier * m_quasistaticStep * MS_20);
        } else {
            // Dynamic is constant voltage
            m_quasistaticVolts = m_quasistaticStep * m_directionMultiplier;
        }

        // TODO REMOVE
        // double angle = m_modFL.getCachedPosition().angle.getDegrees();
        // TODO adjust angle?
        // m_point.withModuleDirection(new Rotation2d(Units.degreesToRadians(-angle)));
        
        setDriveVoltage(m_quasistaticVolts);

        JSONArray data = new JSONArray();
        trackModuleData(data, m_modFL, m_modFR); 
        m_frontTimeEntries.add(data);
        data.clear();
        trackModuleData(data, m_modBL, m_modBR); 
        m_backTimeEntries.add(data);

    }


    @Override
    public boolean isFinished() {
        // TODO see if went 10 ft.   USE: encodertick/TPR/GEAR ratio/tire radius
        // or from odometry
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"distance_diff", m_modFL.getCachedPosition().distanceMeters-m_startMeters);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"distance", m_modFL.getCachedPosition().distanceMeters);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"angle", m_modFL.getCachedPosition().angle.getDegrees());
        // TODO what about FR and back motors
        if(Math.abs(Units.metersToFeet(m_modFL.getCachedPosition().distanceMeters-m_startMeters))>NUMBER_OF_FEET_BEFORE_STOP) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            DriverStation.reportWarning("CHARACTERIZATION TEST INTERRUPTED!"+interrupted, false);
        }
        // STOP driving
        m_quasistaticVolts=0.0;
        setDriveVoltage(m_quasistaticVolts);

        DriverStation.reportWarning("Finished: "+m_jsonObjectName, false);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/"+"END_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());
    }

    public static void writeJsonFile(
        String fileName,
        JSONArray slowFTimeEntries,
        JSONArray slowBTimeEntries,
        JSONArray fastFTimeEntries,
        JSONArray fastBTimeEntries) {
        JSONArray[] timeEntryList = {slowFTimeEntries, slowBTimeEntries, fastFTimeEntries, fastBTimeEntries };
        String timeNames[] = { "slow-forward", "slow-backward", "fast-forward", "fast-backward" };
        JSONObject jsonObject = new JSONObject();
        int ii=0;

        if(slowFTimeEntries.isEmpty()) {
            DriverStation.reportWarning("Missing Slow-Forward timings", false);
        }
        if(slowBTimeEntries.isEmpty()) {
            DriverStation.reportWarning("Missing Slow-Back timings", false);
        }
        if(fastFTimeEntries.isEmpty()) {
            DriverStation.reportWarning("Missing Fast-Forward timings", false);
        }
        if(fastBTimeEntries.isEmpty()) {
            DriverStation.reportWarning("Missing Fast-Back timings", false);
        }
        for(JSONArray timeEntry : timeEntryList) {
            // Write out each time entries
            jsonObject.put(timeNames[ii++], timeEntry);
        }

        jsonObject.put("sysid", true);
        jsonObject.put("test", "Drivetrain");
        jsonObject.put("units", UNITS);
        jsonObject.put("unitsPerRotation", UNITS_PER_ROTATIONS);

        try {
            String folder = DataLogManager.getLogDir();
            if(folder.isEmpty()) {
                folder = ".";
            }
            FileWriter file = new FileWriter(folder+"/"+fileName);
            file.write(jsonObject.toJSONString());
            file.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        DriverStation.reportWarning("JSON file created: "+fileName, false);
    }
    
}
