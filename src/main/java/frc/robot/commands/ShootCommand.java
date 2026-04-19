package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public final ShooterSubsystem shooterSubsystem;
    public Supplier<Double> rpm;

    public ShootCommand(ShooterSubsystem sSub, Supplier<Double> rpmS){
        shooterSubsystem = sSub;
        rpm = rpmS;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetRPM = rpm.get();

        shooterSubsystem.setVelocity(targetRPM);
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){
    }
}
