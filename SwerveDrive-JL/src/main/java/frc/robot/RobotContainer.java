package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private PS4Controller controller = new PS4Controller(0);
  private SwerveSubsystem swerve = new SwerveSubsystem();
 private final SwerveInputStream driveAngularVelocity =
      swerve.getAngularVelocityStream(
                () -> -controller.getLeftY() * Constants.SwerveConstants.MAX_SPEED,
                () -> -controller.getLeftX() * Constants.SwerveConstants.MAX_SPEED,
                () -> -controller.getRightX() * Constants.SwerveConstants.MAX_ANGULAR_SPEED)
            .withAllianceRelativeControl();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}