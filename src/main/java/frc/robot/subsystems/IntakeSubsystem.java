package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(Constants.CANids.intakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    public IntakeSubsystem(){
        config.smartCurrentLimit(30, 20);
    }

    public Command intake(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.intakeSpeed);
        }, this);
    }

    public Command outtake(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.outtakeSpeed);
        }, this);
    }

    public Command stop(){
        return Commands.run(()->{
            motor.set(0);
        }, this);
    }
}
