package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import limelight.networktables.PoseEstimate;
import edu.wpi.first.wpilibj.Timer;

public class RobotMath {
    public static double calculateClosestDiagonal(double rawCurrentAngle){
        double currentAngle = rawCurrentAngle > 0 ? rawCurrentAngle : 360 + rawCurrentAngle;
        if(currentAngle >= 0 && currentAngle <90)return 45;
        if(currentAngle >= 90 && currentAngle < 180)return 135;
        if(currentAngle >= 180 && currentAngle <270)return 225;
        else{return 315;}
    }

    public static Translation2d getHubPosition(){
        Translation2d HubPosition;
        if(DriverStation.getAlliance().isPresent()){
        HubPosition =  DriverStation.getAlliance().get() == Alliance.Red ? Constants.FieldConstants.rHub_POSE.toTranslation2d() : Constants.FieldConstants.bHub_POSE.toTranslation2d();}
        else{HubPosition = Constants.FieldConstants.bHub_POSE.toTranslation2d();}
        return HubPosition;
    }

     public static double DistanceToHub(Pose2d currentPose){
        Translation2d HubPosition = getHubPosition();
        double staticDistance = Math.sqrt(Math.pow((HubPosition.getX() - currentPose.getX()), 2) + Math.pow((HubPosition.getY() - currentPose.getY()), 2));
        return staticDistance;
    }

    public static Translation2d getOutpostPosition(){
        Translation2d OutpostPosition;
        if(DriverStation.getAlliance().isPresent()){
        OutpostPosition =  DriverStation.getAlliance().get() == Alliance.Red ? Constants.FieldConstants.rOutpost_POSE.toTranslation2d() : Constants.FieldConstants.bOutpost_POSE.toTranslation2d();}
        else{OutpostPosition = Constants.FieldConstants.bOutpost_POSE.toTranslation2d();}
        return OutpostPosition;
    }

     public static double DistanceToOutpost(Pose2d currentPose){
        Translation2d OutpostPosition = getHubPosition();
        double staticDistance = Math.sqrt(Math.pow((OutpostPosition.getX() - currentPose.getX()), 2) + Math.pow((OutpostPosition.getY() - currentPose.getY()), 2));
        return staticDistance;
    }

    public static Rotation2d AbsoluteAngleToHub(Pose2d currentPose){
        Rotation2d targetAngle = getHubPosition().minus(currentPose.getTranslation()).getAngle();
        return targetAngle;
    }

    public static boolean hubActive() {
        double matchTime = Timer.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        
        boolean weGoInactiveFirst = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && !gameData.isEmpty()) {
            char inactiveFirst = gameData.charAt(0);
            weGoInactiveFirst = (alliance.get() == DriverStation.Alliance.Red && inactiveFirst == 'R') ||
                                (alliance.get() == DriverStation.Alliance.Blue && inactiveFirst == 'B');
        }

        boolean activeNow = isHubActiveAtTime(matchTime, weGoInactiveFirst);
        
        return activeNow;
    }

    public static boolean activeLater(){
        double matchTime10 = Timer.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        
        boolean weGoInactiveFirst = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && !gameData.isEmpty()) {
            char inactiveFirst = gameData.charAt(0);
            weGoInactiveFirst = (alliance.get() == DriverStation.Alliance.Red && inactiveFirst == 'R') ||
                                (alliance.get() == DriverStation.Alliance.Blue && inactiveFirst == 'B');
        }

        boolean activeNow = isHubActiveAtTime(matchTime10+10, weGoInactiveFirst);
        
        return activeNow;
    }

    private static boolean isHubActiveAtTime(double time, boolean weGoInactiveFirst) {
        if (time > 130 || (time <= 30 && time > 0)) return true; 
        
        int shift;
        if (time > 105)      shift = 1; 
        else if (time > 80)  shift = 2; 
        else if (time > 55)  shift = 3; 
        else                 shift = 4; 

        return (shift == 1 || shift == 3) ? !weGoInactiveFirst : weGoInactiveFirst;
    }

    public static Matrix<N3, N1> getDynamicStdDevs(PoseEstimate estimate, double baseSD){
        double dist = estimate.avgTagDist;
        double tagCountMultiplier = estimate.tagCount > 1 ? 1 : 0.5;
        double multiplier = (1.0 + 0.1 * Math.pow(dist, 2))*tagCountMultiplier;

        return VecBuilder.fill(baseSD * multiplier, baseSD * multiplier, 9999999);
    }
}