package frc.robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

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

  public static class FieldConstants{
    public static final Translation3d rHUB_POSE = new Translation3d(11.938, 4.034536, 1.5748);
    public static final Translation3d bHub_POSE = new Translation3d(4.5974, 4.034536, 1.5748);
  }
}
