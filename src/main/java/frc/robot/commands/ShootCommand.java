package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public final ShooterSubsystem shooterSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final DriveSubsystem driveSubsystem;
    Supplier<Double> offset;

    public ShootCommand(ShooterSubsystem sSub, IndexerSubsystem iSub, DriveSubsystem dSub, Supplier<Double> rpmOffset){
        shooterSubsystem = sSub;
        indexerSubsystem = iSub;
        driveSubsystem = dSub;
        offset = rpmOffset;

        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double hubDist = RobotMath.DistanceToHub(driveSubsystem.getPose());
        double targetRPM = Constants.ShooterMap.shooterMap.get(hubDist) + offset.get();

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
    }
}
