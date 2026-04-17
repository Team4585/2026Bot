package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeltFloorSubsystem extends SubsystemBase{
    public SparkMax motor = new SparkMax(Constants.CANids.beltFloorID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    public BeltFloorSubsystem(){
        config.smartCurrentLimit(20, 10);
    }

    public void enable(){
        motor.set(Constants.SpeedConstants.beltFloorSpeed);
    }

    public void stop(){
        motor.set(0);
    }
}
