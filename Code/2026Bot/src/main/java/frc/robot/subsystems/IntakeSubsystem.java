package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class IntakeSubsystem {
    private SparkMax motor = new SparkMax(Constants.CANids.intakeMotorID, MotorType.kBrushless);

    public Command intake(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.intakeSpeed);
        });
    }

    public Command outtake(){
        return Commands.run(()->{
            motor.set(Constants.SpeedConstants.outtakeSpeed);
        });
    }
}
