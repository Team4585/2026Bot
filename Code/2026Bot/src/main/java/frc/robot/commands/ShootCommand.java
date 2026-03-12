package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SOTM;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public final DriveSubsystem driveSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    public final IndexerSubsystem indexerSubsystem;

    public ShootCommand(DriveSubsystem dSub, ShooterSubsystem sSub, IndexerSubsystem iSub){
        driveSubsystem = dSub;
        shooterSubsystem = sSub;
        indexerSubsystem = iSub;

        addRequirements(driveSubsystem, shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize(){
        SOTM.Config sotmConfig = new SOTM.Config();
        sotmConfig.launcherOffsetX = Constants.OffsetConstants.shooterXOffset;
        sotmConfig.launcherOffsetY = Constants.OffsetConstants.shooterYOffset;
        sotmConfig.phaseDelayMs = 30.0;
        sotmConfig.mechLatencyMs = 20.0;
        sotmConfig.maxTiltDeg = 5.0;
        sotmConfig.headingSpeedScalar = 1.0; 
        sotmConfig.headingReferenceDistance = 2.5;

        SOTM sotmSolver = new SOTM(sotmConfig);
    }

    @Override
    public void execute(){
        

    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean interrupted){}
}
