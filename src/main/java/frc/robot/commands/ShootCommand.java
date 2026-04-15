package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltFloorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public final ShooterSubsystem shooterSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final BeltFloorSubsystem beltFloorSubsystem; 
    public Supplier<Double> rpm;

    public ShootCommand(ShooterSubsystem sSub, IndexerSubsystem iSub, BeltFloorSubsystem bSub, Supplier<Double> rpmS){
        shooterSubsystem = sSub;
        indexerSubsystem = iSub;
        beltFloorSubsystem = bSub;
        rpm = rpmS;

        addRequirements(shooterSubsystem, indexerSubsystem, beltFloorSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetRPM = rpm.get();
        beltFloorSubsystem.enable();

        shooterSubsystem.setVelocity(targetRPM);
        if(shooterSubsystem.ready()){
            indexerSubsystem.enable();
        }
        else{
            indexerSubsystem.push();
        }
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){
        indexerSubsystem.stop();
        beltFloorSubsystem.stop();
    }
}
