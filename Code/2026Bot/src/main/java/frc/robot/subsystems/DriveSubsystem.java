// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMath;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public DriveSubsystem() {
     try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(14.44542));
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command bumpRotation(DoubleSupplier TranslationX, DoubleSupplier TranslationY){
    Rotation2d currentRotation = swerveDrive.getOdometryHeading();
    double currentAngle = currentRotation.getDegrees();

    double target = RobotMath.calculateClosestDiagonal(currentAngle);

    ChassisSpeeds rtargetSpeeds = swerveDrive.swerveController.getTargetSpeeds(TranslationX.getAsDouble()*swerveDrive.getMaximumChassisVelocity(), TranslationY.getAsDouble()*swerveDrive.getMaximumChassisVelocity(),Math.cos(Math.toRadians(target)), Math.sin(Math.toRadians(target)), swerveDrive.getOdometryHeading().getRadians(),swerveDrive.getMaximumChassisVelocity());
    return run(()->{
      driveFieldOriented(()->rtargetSpeeds);
    });
  }






  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive.updateOdometry();
  }
}
