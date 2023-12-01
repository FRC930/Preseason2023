// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.json.simple.JSONArray;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CharacterizeAutoCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

/** Add your docs here. */
public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    JSONArray m_slowFTimeEntriesForFront=new JSONArray();
    JSONArray m_slowBTimeEntriesForFront=new JSONArray();
    JSONArray m_fastFTimeEntriesForFront=new JSONArray();
    JSONArray m_fastBTimeEntriesForFront=new JSONArray();
    JSONArray m_slowFTimeEntriesForBack=new JSONArray();
    JSONArray m_slowBTimeEntriesForBack=new JSONArray();
    JSONArray m_fastFTimeEntriesForBack=new JSONArray();
    JSONArray m_fastBTimeEntriesForBack=new JSONArray();


    public AutoCommandManager(boolean characterizeRobot, SwerveDrivetrainSubsystem drivetrain) {
        // slow-forward
        Command charSlowForwardCommand = new CharacterizeAutoCommand(drivetrain,true,false,m_slowFTimeEntriesForFront,m_slowFTimeEntriesForBack);
        // slow-back
        Command charSlowBackCommand = new CharacterizeAutoCommand(drivetrain,false,false,m_slowBTimeEntriesForFront,m_slowBTimeEntriesForBack);
        // fast-forward
        Command charFastForwardCommand = new CharacterizeAutoCommand(drivetrain,true,true,m_fastFTimeEntriesForFront,m_fastFTimeEntriesForBack);
        // fast-back
        Command charFastBackCommand = new CharacterizeAutoCommand(drivetrain,false,true,m_fastBTimeEntriesForFront,m_fastBTimeEntriesForBack);
        String date = new SimpleDateFormat("yyyy-MM-dd-hh-mm").format(new Date());
        Command writeJsonFileCommand = new InstantCommand(() -> {
            CharacterizeAutoCommand.writeJsonFile("sysid_data_"+date+".json",
            m_slowFTimeEntriesForFront,m_slowBTimeEntriesForFront,
            m_fastFTimeEntriesForFront,m_fastBTimeEntriesForFront);
            CharacterizeAutoCommand.writeJsonFile("sysid_data_back_"+date+".json",
            m_slowFTimeEntriesForBack,m_slowBTimeEntriesForBack,
            m_fastFTimeEntriesForBack,m_fastBTimeEntriesForBack);
        });
        Command initializeDirectionCommand = new InstantCommand(() -> {
            CharacterizeAutoCommand.initializeDirection(drivetrain);
        });
    
        String pathName = "S_Auto";
        Command planPlanCommand = new PathPlannerAuto(pathName);

        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("PathPlanner-S_Auto", planPlanCommand);
        if(characterizeRobot) {
            m_chooser.addOption("Initalize-Direction", initializeDirectionCommand);
            m_chooser.addOption("Slow-Forward", charSlowForwardCommand);
            m_chooser.addOption("Slow-Back", charSlowBackCommand);
            m_chooser.addOption("Fast-Forward", charFastForwardCommand);
            m_chooser.addOption("Fast-Back", charFastBackCommand);
            m_chooser.addOption("Write-File", writeJsonFileCommand);

            // TODO CHARACTERIZATION: FLUDGED to zero out drivegains 
            // NOTE: MAY NOT need to since only used in closed loop
            drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(new Slot0Configs());
            drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(new Slot0Configs());
            drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(new Slot0Configs());
            drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(new Slot0Configs());
        }

        SmartDashboard.putData("SelectAuto", m_chooser);
    }

    public SendableChooser<Command> getAutoManagerChooser() {
        return m_chooser;
    }
      
    public Command getAutoManagerSelected() {
        return m_chooser.getSelected();
    }  
}
