package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(Constants.CANids.indexerMotorID, MotorType.kBrushless);

    public Command enable(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.indexSpeed);
        });
    }

    public Command stop(){
        return Commands.run(()->{
            motor.set(0);
        });
    }

    public Command push(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.indexPushSpeed);
        });
    }
}
