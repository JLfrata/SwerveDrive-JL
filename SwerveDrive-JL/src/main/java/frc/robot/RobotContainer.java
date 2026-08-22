// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private PS4Controller controller = new PS4Controller(0);
  private SwerveSubsystem swerve = new SwerveSubsystem();
  private SwerveInputStream driveAngularVelocity = swerve.driveAngularVelocity(swerve, controller);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.driveRobotRelative(driveAngularVelocity));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
