package frc.robot;
import edu.wpi.first.math.geometry.Pose3d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.05;
  }

  public static class VisionConstants {
    public static final String ll3_hostname = "limelight-3";
    public static final String ll4_hostname = "limelight-4";
    public static final String ll3a_hostname = "limelight-3a";

    public static final Pose3d ll3_offset = Pose3d.kZero;
    public static final Pose3d ll4_offset = Pose3d.kZero;
    public static final Pose3d ll3a_offset = Pose3d.kZero;
  }
}
