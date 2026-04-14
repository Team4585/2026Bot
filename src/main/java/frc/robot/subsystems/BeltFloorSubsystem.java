package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeltFloorSubsystem extends SubsystemBase{
    public SparkMax motor = new SparkMax(Constants.CANids.beltFloorID, MotorType.kBrushless);

    public void enable(){
        motor.set(Constants.SpeedConstants.beltFloorSpeed);
    }

    public void stop(){
        motor.set(0);
    }
}
