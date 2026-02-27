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

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import limelight.networktables.LimelightSettings.LEDMode;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  Limelight ll4 ;
  Limelight ll2 ;
  Limelight ll3a;
  LimelightPoseEstimator ll4Estimator;
  LimelightPoseEstimator ll2Estimator;
  LimelightPoseEstimator ll3aEstimator;
  boolean ll4_attached;
  boolean ll2_attached;
  boolean ll3a_attached;
  boolean odometryOff;
  RobotConfig autoConfig; 

  public DriveSubsystem() {
    //swerve drive
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
     try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(14.44542), new Pose2d(4,4, Rotation2d.kZero));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.synchronizeModuleEncoders();
    swerveDrive.setModuleStateOptimization(true);
   
    //vision
    try{
   ll4 = new Limelight(Constants.VisionConstants.ll4_hostname);
      ll4.getSettings()
   .withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Constants.VisionConstants.ll4_offset).save();
   ll4Estimator = new LimelightPoseEstimator(ll4, EstimationMode.MEGATAG2);
   ll4_attached = true;
    }
    catch(Exception e){
      System.out.println("Limelight 4 not found.");
      ll4_attached = false;
    }

    try{
   ll2 = new Limelight(Constants.VisionConstants.ll2_hostname);
   ll2.getSettings()
   .withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Constants.VisionConstants.ll2_offset).save();
   ll2Estimator = new LimelightPoseEstimator(ll2, EstimationMode.MEGATAG2);
   ll2_attached = true;
    }
    catch(Exception e){
      System.out.println("Limelight 3 not found");
      ll2_attached = false;
    }

    try{
   ll3a = new Limelight(Constants.VisionConstants.ll3a_hostname);
   ll3a.getSettings()
   .withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Constants.VisionConstants.ll3a_offset).save();
   ll3aEstimator = new LimelightPoseEstimator(ll3a, EstimationMode.MEGATAG2);
   ll3a_attached = true;
    }
    catch(Exception e){
      System.out.println("Limelight 3A not found");
      ll3a_attached = false;
    }
      
    if(ll4_attached || ll2_attached || ll3a_attached){
      swerveDrive.stopOdometryThread();
      odometryOff = true;
    }

    else{odometryOff = false;}

   Limelight.setupPortForwardingUSB(0);

    //pathplanner
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
  public Command driveRobotOriented(ChassisSpeeds speeds){
    return run(() -> {
      swerveDrive.drive(speeds);
    });
  }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command bumpRotation(DoubleSupplier TranslationX, DoubleSupplier TranslationY){
    return run(()->{
      Rotation2d currentRotation = swerveDrive.getOdometryHeading();
    double currentAngle = currentRotation.getDegrees();

    double target = RobotMath.calculateClosestDiagonal(currentAngle);

    ChassisSpeeds rtargetSpeeds = swerveDrive.swerveController.getTargetSpeeds(TranslationY.getAsDouble()*swerveDrive.getMaximumChassisVelocity(), TranslationX.getAsDouble()*swerveDrive.getMaximumChassisVelocity(),Math.cos(Math.toRadians(target)), Math.sin(Math.toRadians(target)), swerveDrive.getOdometryHeading().getRadians(),swerveDrive.getMaximumChassisVelocity());
    
      swerveDrive.driveFieldOriented(rtargetSpeeds);
    });
  }

   public Command brake(){
    return run(()->{
      swerveDrive.lockPose();
    });
  }

  public Command pointToHeading(DoubleSupplier TranslationX, DoubleSupplier TranslationY){
    return run(()->{
      double targetHeading = Math.atan2(TranslationY.getAsDouble(), TranslationX.getAsDouble());
      ChassisSpeeds ptargetSpeeds = swerveDrive.swerveController.getTargetSpeeds(TranslationY.getAsDouble(), TranslationX.getAsDouble(), -Math.cos(targetHeading), -Math.sin(targetHeading), swerveDrive.getOdometryHeading().getRadians(),swerveDrive.getMaximumChassisVelocity());
      swerveDrive.driveFieldOriented(ptargetSpeeds);
    });
  }


//other
  private boolean validateMeasurement(PoseEstimate estimate){
     if (estimate.avgTagDist < 4 && estimate.tagCount > 0 && estimate.getMinTagAmbiguity() < 0.3){
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
    if(ll2_attached)ll2.getSettings().withRobotOrientation(robotOrientation).save();
    if(ll3a_attached)ll3a.getSettings().withRobotOrientation(robotOrientation).save();

  Optional<PoseEstimate> visionEstimate_ll4 = Optional.empty();
  Optional<PoseEstimate> visionEstimate_ll2 = Optional.empty();
  Optional<PoseEstimate> visionEstimate_ll3a = Optional.empty();

   if(ll4_attached){visionEstimate_ll4 = ll4Estimator.getAlliancePoseEstimate();}
   if(ll2_attached){visionEstimate_ll2 = ll2Estimator.getAlliancePoseEstimate();}

   if(ll3a_attached){
  var lla3a_results = ll3a.getLatestResults();
  if (lla3a_results.isPresent()) {
      int ll3aPipeline = (int) lla3a_results.get().pipelineID;
      if (ll3aPipeline == 0) {
          visionEstimate_ll3a = ll3aEstimator.getAlliancePoseEstimate();
      }
  }}

    if(visionEstimate_ll4.isPresent()){
      if(validateMeasurement(visionEstimate_ll4.get())){
        swerveDrive.addVisionMeasurement(visionEstimate_ll4.get().pose.toPose2d(), visionEstimate_ll4.get().timestampSeconds);
      }
    }

    if(visionEstimate_ll2.isPresent()){
      if(validateMeasurement(visionEstimate_ll2.get())){
        swerveDrive.addVisionMeasurement(visionEstimate_ll2.get().pose.toPose2d(), visionEstimate_ll2.get().timestampSeconds);
      }
    }

    if(visionEstimate_ll3a.isPresent()){
      if(validateMeasurement(visionEstimate_ll3a.get())){
        swerveDrive.addVisionMeasurement(visionEstimate_ll3a.get().pose.toPose2d(), visionEstimate_ll3a.get().timestampSeconds);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // swerveDrive.updateOdometry();
  }
}
