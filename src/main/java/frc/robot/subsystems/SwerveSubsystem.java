package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    setupPathPlanner();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      AutoBuilder.configure(swerveDrive::getPose, swerveDrive::resetOdometry, swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(speedsRobotRelative, swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          }, new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)), config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }, this);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }
}
