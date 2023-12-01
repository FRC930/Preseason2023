 package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class CharacterizeAutoCommand extends Command {

    private SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
    private SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

    private static final double MS_20 = .02; // TODO is this 20 ms
    private static final int RADIUS = 2; // INches
    //"Meters", "Feet", "Inches", "Radians", "Rotations", and "Degrees".
    private static final String UNITS = "Rotations"; 
    private static final double UNITS_PER_ROTATIONS = 1.0; // TODO detmine unit per 
    // private static final String UNITS = "Inches"; 
    // private static final double UNITS_PER_ROTATIONS = RADIUS*2*Math.PI; // TODO detmine unit per 
    private static final double SLOW_QUASISTIC_STEP = 0.25; // TODO what to set
    private static final double FAST_DYNAMIC_STEP = 6.0;

    // TODO USE VALUES (rotations)
    private static final double TICKS_PER_ROTATION = 2048.0;
    private static final double GEAR_RATIO = 8.16; // 6.86 


    private String m_jsonObjectName;
    private double m_directionMultiplier=1.0;
    private double m_quasistaticStep = SLOW_QUASISTIC_STEP; 
    private JSONArray m_frontTimeEntries = null;
    private JSONArray m_backTimeEntries = null;
    private SwerveDrivetrain m_drivetrain;
    private SwerveModule m_modFL;
    private SwerveModule m_modFR;
    private SwerveModule m_modBL;
    private SwerveModule m_modBR;
    private double m_quasistaticVolts = 0.0;
    private double m_startMeters;
    private int m_loopCnt = 0;
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
     * - how to get simulation consistantly working with it
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
        JSONArray backTimeEntries)
    {
        m_frontTimeEntries = frontTimeEntries;
        m_backTimeEntries = backTimeEntries;
        m_drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        String stageSpeed;
        if(fastSpeed) {
            m_quasistaticStep = FAST_DYNAMIC_STEP;
            stageSpeed="fast";
        } else {
            m_quasistaticStep = SLOW_QUASISTIC_STEP;
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

        // TODO straight?? (maybe apply correction)
        // m_modFL.getSteerMotor().setVoltage(0.0);
        // m_modFR.getSteerMotor().setVoltage(0.0);
        // m_modBL.getSteerMotor().setVoltage(0.0);
        // m_modBR.getSteerMotor().setVoltage(0.0);
    }
    
    static public void initializeDirection(SwerveDrivetrainSubsystem driveTrain) {
        // driveTrain.tareEverything();

    }
    public void initializeDirectionNonStatic() {
        // m_point.withModuleDirection(new Rotation2d(Units.degreesToRadians(0.0)));
        // m_forwardStraight.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
        // m_drivetrain.seedFieldRelative(new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(0.0))));
        // m_drivetrain.tareEverything(); 
        // Thread.currentThread();
        // try {
        //     Thread.sleep(2000);
        // } catch (InterruptedException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }

    }

    @Override
    public void initialize() {
        System.out.println(m_jsonObjectName+" Starting...");
        m_quasistaticVolts=0.0;
        setDriveVoltage(m_quasistaticVolts);

        // TODO HOW TO GET STEER wheel to be in initial position!!!!
        initializeDirectionNonStatic();
    
        SmartDashboard.putNumber("START_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("END_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());

        // TODO log start of 
        m_frontTimeEntries.clear();
        m_backTimeEntries.clear();
        m_loopCnt = 0;
        m_startMeters = (m_modFL.getCachedPosition().distanceMeters);
    }


    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            System.out.println("INTERRUPTED!"+interrupted);
        }
        m_quasistaticVolts=0.0;
        setDriveVoltage(m_quasistaticVolts);
        System.out.println("Finished: "+m_jsonObjectName);
        SmartDashboard.putNumber("END_DEGREES", m_drivetrain.getState().Pose.getRotation().getDegrees());
        // writeJsonFile();
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

        for(JSONArray timeEntry : timeEntryList) {
            // Write out each time entries
            jsonObject.put(timeNames[ii++], timeEntry);
        }

        jsonObject.put("sysid", true);
        jsonObject.put("test", "Drivetrain");
        jsonObject.put("units", UNITS);
        jsonObject.put("unitsPerRotation", UNITS_PER_ROTATIONS);

        // System.out.println(jo.toJSONString());

        try {
            String folder = "/home/lvuser/";
            if(Utils.isSimulation()) {
                folder = "./";
            }
            FileWriter file = new FileWriter(folder+fileName);
            file.write(jsonObject.toJSONString());
            file.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("JSON file created: "+fileName);
    }

    public double convertRotationToUnits(double rotations) {
        // TODO is this how to convert? WHAT ABOUT TICKS_PER_ROTATION
        // return rotations/(GEAR_RATIO*UNITS_PER_ROTATIONS);
        //return rotations*UNITS_PER_ROTATIONS/GEAR_RATIO;
        return rotations;
    }

    @Override
    public void execute() {
        // Increase power to drive
        if(m_quasistaticStep != FAST_DYNAMIC_STEP) {
            // TODO which way is correct
            m_quasistaticVolts = m_quasistaticVolts + (m_directionMultiplier * m_quasistaticStep * MS_20);
            // m_quasistaticVolts = m_quasistaticVolts + (m_directionMultiplier * m_quasistaticStep);
        } else {
            m_quasistaticVolts = m_quasistaticStep * m_directionMultiplier;
        }


        double angle = m_modFL.getCachedPosition().angle.getDegrees();
        // TODO adjust angle?
        // m_point.withModuleDirection(new Rotation2d(Units.degreesToRadians(-angle)));
        
        setDriveVoltage(m_quasistaticVolts);
        System.out.println("volts "+m_quasistaticVolts);

        JSONArray data = new JSONArray();
        trackModuleData(data, m_modFL, m_modFR); 
        m_frontTimeEntries.add(data);
        data.clear();
        trackModuleData(data, m_modBL, m_modBR); 
        m_backTimeEntries.add(data);

    }

    private void trackModuleData(JSONArray data, SwerveModule leftModule, SwerveModule rightModule) {
        // TODO log each time
        // TODO timestamp 1,
        data.add(Timer.getFPGATimestamp()); // TODO DOUBLE
        // TODO l voltage 1,
        data.add(leftModule.getDriveMotor().getMotorVoltage().getValue()); 
        // TODO r voltage 1,
        data.add(rightModule.getDriveMotor().getMotorVoltage().getValue()); 
        // TODO l position 1,
        data.add(convertRotationToUnits(leftModule.getDriveMotor().getPosition().getValue())); 
        // TODO  r position 1,
        data.add(convertRotationToUnits(rightModule.getDriveMotor().getPosition().getValue())); 
        // TODO l velocity 1,
        data.add(convertRotationToUnits(leftModule.getDriveMotor().getVelocity().getValue())); 
        // TODO r velocity 1,
        data.add(convertRotationToUnits(rightModule.getDriveMotor().getVelocity().getValue())); 
        // TODO angle 1,
        data.add(0.0); 
        // angular rate 1
        data.add(0.0);
    }


    @Override
    public boolean isFinished() {
        // TODO see if went 10 ft.   USE: encodertick/TPR/GEAR ratio/tire radius
        // or from odometry
        //System.out.println("ISFINISH: "+m_quasistaticVolts);
        SmartDashboard.putNumber("distance_diff", m_modFL.getCachedPosition().distanceMeters-m_startMeters);
        SmartDashboard.putNumber("distance", m_modFL.getCachedPosition().distanceMeters);
        SmartDashboard.putNumber("angle", m_modFL.getCachedPosition().angle.getDegrees());
        if(true) {
            // TODO what about FR and back motors
            if(Math.abs(Units.metersToFeet(m_modFL.getCachedPosition().distanceMeters-m_startMeters))>10.0) {
                return true;
            } else {
                return false;
            }

        }else {
            if(m_quasistaticStep == FAST_DYNAMIC_STEP) {
                if(m_loopCnt++ > 100) {
                    return true;
                } else {
                    return false;
                }
            } else {
                if (Math.abs(m_quasistaticVolts) > 2.2){
                    System.out.println("Finished with voltage: "+m_quasistaticVolts);
                    return true;
                } else {
                    return false;
                }
            }
        }
    }
    
}
