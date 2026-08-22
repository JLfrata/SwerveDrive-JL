// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.io.File;
import java.io.InputStream;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import swervelib.parser.deserializer.ReflectionsManager.Gyro;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");

  private Field2d field = new Field2d();
  private static final Pose2d bluePose = new Pose2d(3.560, 4.035, new Rotation2d(0));
  private static final Pose2d redPose = new Pose2d(12.977, 4.035, new Rotation2d(Math.toRadians(0)));

  SwerveDrive swerveDrive;
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  CANcoder frontleft = new CANcoder(11);
  CANcoder frontright = new CANcoder(12);
  CANcoder backleft = new CANcoder(14);
  CANcoder backright = new CANcoder(13);
  // PigeonIMU pigeonIMU = new PigeonIMU(20);

  public SwerveSubsystem() {
    var cfg = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(0, 0, Rotation2d.kZero))
        .withSubsystem(this)
        .withTelemetry(TelemetryVerbosity.HIGH)
        .withTranslationController(new PIDController(1.0, 0, 0)) // input: meters of position error
        .withRotationController(new PIDController(1.0, 0, 0)) // input: radians of heading error
        .withGyro(() -> Degrees.of(-gyro.getAngle()))
        .withGyroInverted(false);
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
          .createSwerveDrive(cfg);

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(swerveDrive, x, y, rot);
  }

  public SwerveInputStream driveAngularVelocity(SwerveSubsystem swerve, PS4Controller controller) {
    swerve.getAngularVelocityStream(
        () -> -controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> -controller.getRightX())
        .withAllianceRelativeControl();
  }

  public Command driveRobotRelative(SwerveInputStream stream) {
    return swerveDrive.drive(() -> ChassisSpeeds.fromRobotRelativeSpeeds(stream.get(),
        new Rotation2d(swerveDrive.getGyroAngle())));
  }

  // public Command driveTeleOp(DoubleSupplier translationX, DoubleSupplier
  // translationY,
  // DoubleSupplier angularRotationX) {
  // return run(() -> {
  // swerveDrive.drive(new Translation2d(translationX.getAsDouble() *
  // swerveDrive.getMaximumChassisVelocity(),
  // translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
  // angularRotationX.getAsDouble() *
  // swerveDrive.getMaximumChassisAngularVelocity(),
  // true,
  // false);
  // });
  // }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }


  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public boolean exampleCondition() {

    return false;
  }

  public double getCancoderDegrees(CANcoder cancoder) {
    double rotations = cancoder.getAbsolutePosition().getValueAsDouble();
    return rotations * 360.0;
  }

  public Command resetPoseByAlliance() {
    return new InstantCommand(() -> {
      var alliance = DriverStation.getAlliance();
      Pose2d inicialPose = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue)
          ? bluePose
          : redPose;
      this.resetOdometry(inicialPose);
    });
  }

  private void updateDashboardField() {
    Pose2d robotPose = swerveDrive.getPose();

    field.setRobotPose(robotPose);
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    if (swerveDrive != null) {

      SmartDashboard.putNumber("FR CanCoder", getCancoderDegrees(frontright));
      SmartDashboard.putNumber("FL CanCoder", getCancoderDegrees(frontleft));
      SmartDashboard.putNumber("BR CanCoder", getCancoderDegrees(backright));
      SmartDashboard.putNumber("BL CanCoder", getCancoderDegrees(backleft));

      //     swerveDrive.getModules()[0].getAbsolutePosition()); // Front Left

      // SmartDashboard.putNumber("Swerve/Encoder Absoluto FR",
      //     swerveDrive.getModules()[1].getAbsolutePosition()); // Front Right

      // SmartDashboard.putNumber("Swerve/Encoder Absoluto BL",
      //     swerveDrive.getModules()[2].getAbsolutePosition()); // Back Left

      // SmartDashboard.putNumber("Swerve/Encoder Absoluto BR",
      //     swerveDrive.getModules()[3].getAbsolutePosition()); // Back Right
      // SmartDashboard.putNumber("X: ", swerveDrive.getPose().getX());
      // SmartDashboard.putNumber("Y: ", swerveDrive.getPose().getY());

      // SmartDashboard.putNumber("Swerve Module velocity 1", swerveDrive.getModules()[0].getDriveMotor().getVelocity());
      // SmartDashboard.putNumber("Swerve Module velocity 2", swerveDrive.getModules()[1].getDriveMotor().getVelocity());
      // SmartDashboard.putNumber("Swerve Module velocity 3", swerveDrive.getModules()[2].getDriveMotor().getVelocity());
      // SmartDashboard.putNumber("Swerve Module velocity 4", swerveDrive.getModules()[3].getDriveMotor().getVelocity());
    }
  }

  @Override
  public void simulationPeriodic() {
  }

}