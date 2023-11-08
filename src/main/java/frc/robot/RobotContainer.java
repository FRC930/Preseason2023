// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class RobotContainer {
  private static final double JOYSTICK_DEADBAND = 0.1;
  private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1;
  private static final double PERCENT_SPEED = 0.3;
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.RobotCentric straightDrive = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * PERCENT_SPEED) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * PERCENT_SPEED) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            .withDeadband(JOYSTICK_DEADBAND)
            .withRotationalDeadband(JOYSTICK_ROTATIONAL_DEADBAND)
        ));

    joystick.pov(0).whileTrue(
      drivetrain.applyRequest(() -> straightDrive.withVelocityX(0.5 * MaxSpeed * PERCENT_SPEED).withVelocityY(0.0)
      ));
    joystick.pov(180).whileTrue(
      drivetrain.applyRequest(() -> straightDrive.withVelocityX(-0.5 * MaxSpeed * PERCENT_SPEED).withVelocityY(0.0)
      ));
    joystick.pov(90).whileTrue(
      drivetrain.applyRequest(() -> straightDrive.withVelocityX(0.0).withVelocityY(-0.5 * MaxSpeed * PERCENT_SPEED)
      ));
    joystick.pov(270).whileTrue(
      drivetrain.applyRequest(() -> straightDrive.withVelocityX(0.0).withVelocityY(0.5 * MaxSpeed * PERCENT_SPEED)
      ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
