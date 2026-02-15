package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotMath {
    public static double calculateClosestDiagonal(double currentAngle){
        if(currentAngle >= 0 && currentAngle <90)return 45;
        if(currentAngle >= 90 && currentAngle <180)return 135;
        if(currentAngle >= 180 && currentAngle <270)return 225;
        else{return 315;}
    }

    public static double DistanceToHub(Pose2d currentPose){
        Translation2d HubPosition;
        if(DriverStation.getAlliance().isPresent()){
        HubPosition =  DriverStation.getAlliance().get() == Alliance.Red ? Constants.FieldConstants.rHub_POSE.toTranslation2d() : Constants.FieldConstants.bHub_POSE.toTranslation2d();}
        else{HubPosition = Constants.FieldConstants.rHub_POSE.toTranslation2d();}
        double staticDistance = Math.sqrt(Math.pow((HubPosition.getX() - currentPose.getX()), 2) + Math.pow((HubPosition.getY() - currentPose.getY()), 2));
        return staticDistance;
    }
}
