package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PassCommand extends Command{
    public final ShooterSubsystem shooterSubsystem;

    public PassCommand(ShooterSubsystem sSub){
        shooterSubsystem = sSub;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetRPM = -5500;

        shooterSubsystem.setVelocity(targetRPM);
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){

    }
}
