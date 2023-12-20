package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
//https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)/Music%20-%20Orchestra/src/main/deploy
public class MusicCommand extends Command {
    Orchestra m_o = new Orchestra();
    public MusicCommand(SwerveDrivetrainSubsystem drivetrain) {
        //drivetrain.getModule(0).getDriveMotor().set(TalonFXControlMode.MusicTone, 123.0);
        m_o.addInstrument(drivetrain.getModule(0).getDriveMotor());
        m_o.addInstrument(drivetrain.getModule(1).getDriveMotor());
        m_o.addInstrument(drivetrain.getModule(2).getDriveMotor());
        m_o.addInstrument(drivetrain.getModule(3).getDriveMotor());
        m_o.loadMusic("song6.chrp");
    }

    @Override
    public void end(boolean interrupted) {
        m_o.stop();
    }

    @Override
    public void initialize() {
        m_o.play();
    }

    

}
