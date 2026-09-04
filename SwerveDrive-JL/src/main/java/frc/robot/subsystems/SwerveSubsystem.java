package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.wpilibj.DriverStation;
import static edu.wpi.first.units.Units.*;
import com.studica.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {

  private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private Field2d field = new Field2d();
  private static final Pose2d bluePose = new Pose2d(3.560, 4.035, new Rotation2d(0));
  private static final Pose2d redPose = new Pose2d(12.977, 4.035, new Rotation2d(0));

  private SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    var cfg = new SwerveDriveConfig()
        .withMaximumChassisSpeed(MetersPerSecond.of(Constants.SwerveConstants.MAX_SPEED), DegreesPerSecond.of(Constants.SwerveConstants.MAX_ANGULAR_SPEED))
        .withStartingPose(new Pose2d(0, 0, Rotation2d.kZero))
        .withSubsystem(this)
        .withTelemetry(TelemetryVerbosity.HIGH)
        .withTranslationController(new PIDController(1.0, 0, 0))
        .withRotationController(new PIDController(1.0, 0, 0))
        .withGyro(() -> Degrees.of(gyro.getAngle()))
        .withGyroInverted(true);
        
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
          .createSwerveDrive(cfg);
    } catch (Exception e) {
      throw new RuntimeException("Erro ao carregar arquivos do Swerve", e);
    }
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(swerveDrive, x, y, rot);
  }
  public void drive(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
          swerveDrive.setFieldRelativeChassisSpeeds(new ChassisSpeeds(
              -translationX.getAsDouble() * Constants.SwerveConstants.MAX_SPEED,
              -translationY.getAsDouble() * Constants.SwerveConstants.MAX_SPEED,
              -angularRotationX.getAsDouble() * swerveDrive.getConfig().getMaximumChassisAngularVelocity().orElseThrow().in(RadiansPerSecond)));
  }

  public Command driveFieldRelative(SwerveInputStream stream) {
    return run(() -> swerveDrive.setFieldRelativeChassisSpeeds(stream.get()));
  }

  public Command driveRobotRelative(SwerveInputStream stream) {
    return swerveDrive.drive(() -> ChassisSpeeds.fromRobotRelativeSpeeds(stream.get(), new Rotation2d()));
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Command resetGyro() {
    return new InstantCommand(() -> {
      gyro.reset();
      swerveDrive.zeroGyro();
    });
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
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

  @Override
  public void periodic() {
    if (swerveDrive != null) {
      swerveDrive.updateTelemetry();
      field.setRobotPose(swerveDrive.getPose());
      SmartDashboard.putData("Field", field);
      SmartDashboard.putNumber("Ângulo FL (Graus)", swerveDrive.getModuleStates()[0].angle.getDegrees());
      SmartDashboard.putNumber("Ângulo FR (Graus)", swerveDrive.getModuleStates()[1].angle.getDegrees());
      SmartDashboard.putNumber("Ângulo BL (Graus)", swerveDrive.getModuleStates()[2].angle.getDegrees());
      SmartDashboard.putNumber("Ângulo BR (Graus)", swerveDrive.getModuleStates()[3].angle.getDegrees());
    }
  }
}