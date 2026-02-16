package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.05;
  }

  public static class VisionConstants {
    public static final String ll3_hostname = "llfour";
    public static final String ll4_hostname = "llthree";
    public static final String ll3a_hostname = "llthreea";

    public static final Pose3d ll3_offset = new Pose3d(
      new Translation3d(0.2425, -0.0302, 0.0605), 
      new Rotation3d(0.0, Math.toRadians(-35), 0.0));
    public static final Pose3d ll4_offset = new Pose3d(
      new Translation3d(0.2728, 0.0051, 0.0710), 
      new Rotation3d(0.0, Math.toRadians(-45.0), 0.0));
    public static final Pose3d ll3a_offset = Pose3d.kZero;
  }

  public static class FieldConstants{
    public static final Translation3d rHub_POSE = new Translation3d(11.938, 4.034536, 1.5748);
    public static final Translation3d bHub_POSE = new Translation3d(4.5974, 4.034536, 1.5748);
  }

  public static class PIDControllers{
    public static final PPHolonomicDriveController autoPID = new PPHolonomicDriveController(new PIDConstants(0.0020645), new PIDConstants(0.006, 0, 0.001));
  }
}
