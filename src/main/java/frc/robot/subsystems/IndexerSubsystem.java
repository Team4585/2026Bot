package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(Constants.CANids.indexerMotorID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    public IndexerSubsystem(){
        config.smartCurrentLimit(20, 10);
    }

    public void enable(){
        motor.set(Constants.SpeedConstants.indexSpeed);
    }

    public void stop(){
        motor.set(0);
    }

    public void push(){
        motor.set(Constants.SpeedConstants.indexPushSpeed);
    }
}
