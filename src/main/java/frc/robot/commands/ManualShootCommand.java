package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCommand extends Command{
    public final ShooterSubsystem shooterSubsystem;

    public ManualShootCommand(ShooterSubsystem sSub){
        shooterSubsystem = sSub;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetRPM = Constants.SetpointConstants.TowerShootSpeed;

        shooterSubsystem.setVelocity(targetRPM);
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){
    }
}
