package frc.robot;

public class RobotMath {
    public static double calculateClosestDiagonal(double currentAngle){
        if(currentAngle >= 0 && currentAngle <90)return 45;
        if(currentAngle >= 90 && currentAngle <180)return 135;
        if(currentAngle >= 180 && currentAngle <270)return 225;
        else{return 315;}
    }
}
