package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double deadband = 0.05;
  }

  public static class VisionConstants {
    public static final String ll2_hostname = "lltwo";
    public static final String ll4_hostname = "llfour";
    public static final String ll3a_hostname = "llta";

    public static final Pose3d ll2_offset = new Pose3d(
      new Translation3d(-0.135, 0.287, 0.058), 
      new Rotation3d(0.0, Math.toRadians(-35), 0.0));
    public static final Pose3d ll4_offset = new Pose3d(
      new Translation3d(0.2728, 0.0051, 0.0710), 
      new Rotation3d(0.0, Math.toRadians(-45.0), 0.0));
    public static final Pose3d ll3a_offset = Pose3d.kZero;
  }

  public static class FieldConstants{
    public static final Translation3d rHub_POSE = new Translation3d(11.938, 4.034536, 1.5748);
    public static final Translation3d bHub_POSE = new Translation3d(4.5974, 4.034536, 1.5748);

    public static final Translation3d rOutpost_POSE = new Translation3d(15.75, 7.25, 0);
    public static final Translation3d bOutpost_POSE = new Translation3d(0.75, 0.75, 0);
  }

  public static class PIDFFControllers{
    public static final PPHolonomicDriveController autoPID = new PPHolonomicDriveController(new PIDConstants(0.0020645), new PIDConstants(0.006, 0, 0.001));
    public static class intakePivotPID{
      public static final double kP = 0.8;
      public static final double kI = 0;
      public static final double kD = 0.1;
    }
    public static final ArmFeedforward intakePivotFF = new ArmFeedforward(0.06, 0.18, 0);

    public static class shooterPID{
      public static final double kP = 0.06;
      public static final double kI = 0;
      public static final double kD = 0.12;
    }

    public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.67, 0.03, 0.05);
  }

  public static class CANids{
    public static final int intakePivotMotorID = 9;
    public static final int intakeMotorID = 10;
    public static final int indexerMotorID = 11;
    public static final int shooterMotor1ID = 12;
    public static final int shooterMotor2ID = 13;
    public static final int candleID = 14;
  }

  public static class SetpointConstants{
    public static class IntakePivotSetpoints{
      public static Angle UpPos = Degrees.of(-35);
      public static Angle DownPos = Degrees.of(74);
    }
    public static double TowerShootSpeed = -3400;
  }

  public static class OffsetConstants{
    public static double intakePivotEncoderOffset = 0.21;
    public static double shooterXOffset = 0.1;
    public static double shooterYOffset = 0.0;
  }

  public static class SpeedConstants{
    public static double intakeSpeed = -1;
    public static double outtakeSpeed = 0.8;
    public static double indexSpeed = 0.19;
    public static double indexPushSpeed = -0.1;
  }

  //in pounds
  public static double shooterUpductedWeight = 3.75;
  public static double shooterAxleWeight = 2.2;

  public static double shooterReadyThreshold = 100;

  public static class ShooterMap{
    public static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    static{
      shooterMap.put(0.0, 0.0);
      shooterMap.put(1.0, 1600.0);
      shooterMap.put(2.1, 1600.0);
      shooterMap.put(3.0, 1785.0);
      shooterMap.put(4.0, 2010.0);
      shooterMap.put(5.0,2225.0);
      shooterMap.put(6.0, 2438.0);
    }

  }
}
