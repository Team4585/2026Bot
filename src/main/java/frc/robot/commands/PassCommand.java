package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMath;
import frc.robot.subsystems.BeltFloorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PassCommand extends Command{
     public final ShooterSubsystem shooterSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final DriveSubsystem driveSubsystem;
    public final BeltFloorSubsystem beltFloorSubsystem; 

    public PassCommand(ShooterSubsystem sSub, IndexerSubsystem iSub, BeltFloorSubsystem bSub, DriveSubsystem dSub){
        shooterSubsystem = sSub;
        indexerSubsystem = iSub;
        driveSubsystem = dSub;
        beltFloorSubsystem = bSub;

        addRequirements(shooterSubsystem, indexerSubsystem, beltFloorSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetDist = RobotMath.DistanceToOutpost(driveSubsystem.getPose());
        double targetRPM = Constants.ShooterMap.shooterMap.get(targetDist);

        shooterSubsystem.setVelocity(targetRPM);
        indexerSubsystem.enable();
        beltFloorSubsystem.enable();
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){
        indexerSubsystem.stop();
        beltFloorSubsystem.stop();
    }
}
