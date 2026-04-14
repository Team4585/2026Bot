// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMath;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  Limelight ll4 ;
  Limelight ll3a;
  LimelightPoseEstimator ll4Estimator;
  LimelightPoseEstimator ll3aEstimator;
  boolean ll4_attached;
  boolean ll3a_attached;
  boolean odometryOff;
  RobotConfig autoConfig; 

  public DriveSubsystem() {
    //swerve drive
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
     try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(14.44542));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.synchronizeModuleEncoders();
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setCosineCompensator(true);
    swerveDrive.setHeadingCorrection(false);

    if(RobotBase.isSimulation()) swerveDrive.resetOdometry(new Pose2d(4, 4, new Rotation2d(0)));
   
    //vision
    try{
   ll4 = new Limelight(Constants.VisionConstants.ll4_hostname);
      ll4.getSettings()
   .withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Constants.VisionConstants.ll4_offset).save();
   ll4Estimator = new LimelightPoseEstimator(ll4, EstimationMode.MEGATAG1);
   ll4_attached = true;
    }
    catch(Exception e){
      System.out.println("Limelight 4 not found.");
      ll4_attached = false;
    }

    try{
   ll3a = new Limelight(Constants.VisionConstants.ll3a_hostname);
   ll3a.getSettings()
   .withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Constants.VisionConstants.ll3a_offset).save();
   ll3aEstimator = new LimelightPoseEstimator(ll3a, EstimationMode.MEGATAG1);
   ll3a_attached = true;
    }
    catch(Exception e){
      System.out.println("Limelight 3A not found");
      ll3a_attached = false;
    }
      
    if(ll4_attached || ll3a_attached){
      swerveDrive.stopOdometryThread();
      odometryOff = true;
    }

    else{odometryOff = false;}

   Limelight.setupPortForwardingUSB(0);
  }

  public void setupPathplanner(){
    try{
      autoConfig = RobotConfig.fromGUISettings();
    }
    catch(Exception e){
      e.printStackTrace();
    }

    AutoBuilder.configure(
    this::getPose, 
    this::resetPose, 
    this::getSpeeds, 
    (speeds, feedforwards) -> driveRobotOriented(speeds), 
    Constants.PIDFFControllers.autoPID, 
    autoConfig, 
    () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            }, 
    this
    );
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

  //methods to get stuff
  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getSpeeds(){
    return swerveDrive.getRobotVelocity();
  }


  //drive related commands
  public void driveRobotOriented(ChassisSpeeds speeds){
      swerveDrive.drive(speeds);
  }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command driveToPose(Pose2d pose){
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) 
                                    );
  }

  public Command pathFindAndFollow(PathPlannerPath path){
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));


    return AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);
  }

  public Command aimToPose(Pose2d pose){
    return run(() -> {
        Rotation2d currentRotation = getPose().getRotation();

        double rotationSpeed = swerveDrive.swerveController.getTargetSpeeds(
            0.0, 0.0, 
            pose.getRotation().getRadians(), 
            currentRotation.getRadians(),
            swerveDrive.getMaximumChassisVelocity()
        ).omegaRadiansPerSecond;

        this.driveRobotOriented(new ChassisSpeeds(0, 0, rotationSpeed));

    }).until(() -> {
        double error = Math.abs(getPose().getRotation().minus(pose.getRotation()).getDegrees());
        return error < 2.0;
    });
  }

  public Command bumpRotation(DoubleSupplier TranslationY, DoubleSupplier TranslationX){
    return run(()->{
      Rotation2d currentRotation = swerveDrive.getOdometryHeading();
    double currentAngle = currentRotation.getDegrees();

    double target = RobotMath.calculateClosestDiagonal(currentAngle);

    ChassisSpeeds rtargetSpeeds = swerveDrive.swerveController.getTargetSpeeds(-TranslationY.getAsDouble()*swerveDrive.getMaximumChassisVelocity(), -TranslationX.getAsDouble()*swerveDrive.getMaximumChassisVelocity(),Math.cos(Math.toRadians(target)), Math.sin(Math.toRadians(target)), swerveDrive.getOdometryHeading().getRadians(),swerveDrive.getMaximumChassisVelocity());
    
      swerveDrive.driveFieldOriented(rtargetSpeeds);
    });
  }

   public Command brake(){
    return run(()->{
      swerveDrive.lockPose();
    });
  }

  public Command pointToHeading(DoubleSupplier TranslationY, DoubleSupplier TranslationX){
    return run(()->{
      double targetHeading = Math.atan2(TranslationY.getAsDouble(), TranslationX.getAsDouble());
      ChassisSpeeds ptargetSpeeds = swerveDrive.swerveController.getTargetSpeeds(-TranslationY.getAsDouble(), -TranslationX.getAsDouble(), -Math.cos(targetHeading), -Math.sin(targetHeading), swerveDrive.getOdometryHeading().getRadians(),swerveDrive.getMaximumChassisVelocity());
      swerveDrive.driveFieldOriented(ptargetSpeeds);
    });
  }


//other
  private boolean validateMeasurement(PoseEstimate estimate){
     if (estimate.avgTagDist < 3 && estimate.tagCount > 0 && estimate.getMinTagAmbiguity() < 0.3){
      return true;
     }
     return false;
  }

  
  public void resetPose(Pose2d target){
    swerveDrive.resetOdometry(target);
  }

  @Override
  public void periodic() {
    if(odometryOff){
      swerveDrive.updateOdometry();
    }

    Orientation3d robotOrientation = new Orientation3d(swerveDrive.getGyroRotation3d(), new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(swerveDrive.getYaw().getDegrees())));

    if(ll4_attached)ll4.getSettings().withRobotOrientation(robotOrientation).save();
    if(ll3a_attached)ll3a.getSettings().withRobotOrientation(robotOrientation).save();

  Optional<PoseEstimate> visionEstimate_ll4 = Optional.empty();
  Optional<PoseEstimate> visionEstimate_ll3a = Optional.empty();

   if(ll4_attached){visionEstimate_ll4 = ll4Estimator.getAlliancePoseEstimate();}
  if(ll3a_attached){visionEstimate_ll3a = ll3aEstimator.getAlliancePoseEstimate();}

  if(ll4_attached){
  if (edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
        ll4.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    } else {
        ll4.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    }
  }
    if(visionEstimate_ll4.isPresent()){
      if(validateMeasurement(visionEstimate_ll4.get())){
        swerveDrive.addVisionMeasurement(visionEstimate_ll4.get().pose.toPose2d(), visionEstimate_ll4.get().timestampSeconds, RobotMath.getDynamicStdDevs(visionEstimate_ll4.get(), 0.3));
      }
    }

    if(visionEstimate_ll3a.isPresent()){
      if(validateMeasurement(visionEstimate_ll3a.get())){
        swerveDrive.addVisionMeasurement(visionEstimate_ll3a.get().pose.toPose2d(), visionEstimate_ll3a.get().timestampSeconds, RobotMath.getDynamicStdDevs(visionEstimate_ll3a.get(), 0.4));
      }
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
