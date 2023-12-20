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
    SendableChooser<Command> m_chooserAuto = new SendableChooser<>();
    SendableChooser<Command> m_chooserCharactization = new SendableChooser<>();
    SendableChooser<Command> m_chooser = null;
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
        Command charSlowForwardCommand = new CharacterizeAutoCommand(drivetrain,
            true,false,
            m_slowFTimeEntriesForFront,m_slowFTimeEntriesForBack
            ,null, null);
        // slow-back
        Command charSlowBackCommand = new CharacterizeAutoCommand(drivetrain,
            false,false,
            m_slowBTimeEntriesForFront,m_slowBTimeEntriesForBack
            ,null, null);
        // fast-forward
        Command charFastForwardCommand = new CharacterizeAutoCommand(drivetrain,
            true,true,
            m_fastFTimeEntriesForFront,m_fastFTimeEntriesForBack
            ,null,null);
        // fast-back
        Command charFastBackCommand = new CharacterizeAutoCommand(drivetrain,
            false,true,
            m_fastBTimeEntriesForFront,m_fastBTimeEntriesForBack
            ,null,null);
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
    
        
        Command sAutoPlanCommand = new PathPlannerAuto("S_Auto");
        Command straightForwardPlanCommand = new PathPlannerAuto("StraightForward");
        Command straightForwardWPosePlanCommand = new PathPlannerAuto("StraightForwardWPose");
        Command chainPlanCommand = new PathPlannerAuto("Chain");
        Command chainWPosePlanCommand = new PathPlannerAuto("ChainWPose");
        Command straightForwardFacingPosePlanCommand = new PathPlannerAuto("straightForwardFacing");
        

        // Add commands to the autonomous command chooser
        m_chooserCharactization.setDefaultOption("None", null);
        m_chooserAuto.setDefaultOption("None", null);
        // if(characterizeRobot) {
            m_chooserCharactization.addOption("Initalize-Direction", initializeDirectionCommand);
            m_chooserCharactization.addOption("Slow-Forward", charSlowForwardCommand);
            m_chooserCharactization.addOption("Slow-Back", charSlowBackCommand);
            m_chooserCharactization.addOption("Fast-Forward", charFastForwardCommand);
            m_chooserCharactization.addOption("Fast-Back", charFastBackCommand);
            m_chooserCharactization.addOption("Write-File", writeJsonFileCommand);
        // } else {
            m_chooserAuto.addOption(sAutoPlanCommand.getName(), sAutoPlanCommand);
            m_chooserAuto.addOption(straightForwardPlanCommand.getName(), straightForwardPlanCommand);
            m_chooserAuto.addOption(straightForwardWPosePlanCommand.getName(), straightForwardWPosePlanCommand);
            m_chooserAuto.addOption(chainPlanCommand.getName(), chainPlanCommand);
            m_chooserAuto.addOption(chainWPosePlanCommand.getName(), chainWPosePlanCommand);
            m_chooserAuto.addOption(straightForwardFacingPosePlanCommand.getName(), straightForwardFacingPosePlanCommand);
        // }

        setChooser(characterizeRobot);
    }

    public void setChooser(boolean characterizeRobot) {
        if(characterizeRobot) {
            m_chooser= m_chooserCharactization;
        } else {
            m_chooser= m_chooserAuto;
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
