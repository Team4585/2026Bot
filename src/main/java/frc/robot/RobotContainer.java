// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PassCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.BeltFloorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final BeltFloorSubsystem beltFloorSubsystem = new BeltFloorSubsystem();

  public SendableChooser<Command> autoChooser;

  double offset = 0;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
   private final CommandXboxController m_operatorController = 
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    SwerveInputStream driveStream = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
                                                                () -> 1 * m_driverController.getLeftY(),
                                                                () -> 1 * m_driverController.getLeftX())
                                                            .withControllerRotationAxis(()-> -1* m_driverController.getRightX())
                                                            .deadband(OperatorConstants.deadband)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);


  public RobotContainer() {
    configureAutos();
    configureBindings();
  }

  private void configureAutos(){
      NamedCommands.registerCommand("Orient To Hub", 
        Commands.deferredProxy(() -> {
            Pose2d currentPose = driveSubsystem.getPose();
            return driveSubsystem.aimToPose(new Pose2d(
                currentPose.getTranslation(),
                RobotMath.AbsoluteAngleToHub(currentPose)
            ));
        }).withTimeout(Seconds.of(2))
     );
      NamedCommands.registerCommand("Intake Down", intakePivotSubsystem.pivotDown().withTimeout(Seconds.of(0.01)));
      NamedCommands.registerCommand("Intake Start", intakeSubsystem.intake().withTimeout(Seconds.of(5)));
      NamedCommands.registerCommand("Intake Stop", intakeSubsystem.stop().withTimeout(Seconds.of(0.01)));
      NamedCommands.registerCommand("Pathfind to Depot and Drive Through", Commands.deferredProxy(
        ()->{
          PathPlannerPath path;
                try{
        path  = PathPlannerPath.fromPathFile("Depot Intake");
       }
        catch(Exception e){path = null;}
          return driveSubsystem.pathFindAndFollow(path).withTimeout(Seconds.of(5));
        }
      ));

      NamedCommands.registerCommand("Pathfind to Trench and Drive Through Neutral", Commands.deferredProxy(
        ()->{
          PathPlannerPath path;
                try{
        path  = PathPlannerPath.fromPathFile("Neutral Sweep");
       }
        catch(Exception e){path = null;}
          return driveSubsystem.pathFindAndFollow(path).withTimeout(Seconds.of(15));
        }
      ));


      driveSubsystem.setupPathplanner();

      autoChooser = AutoBuilder.buildAutoChooser();

      SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveStream);

    driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.rightTrigger().whileTrue(driveSubsystem.aimToPose(new Pose2d(
                driveSubsystem.getPose().getTranslation(),
                RobotMath.AbsoluteAngleToHub(driveSubsystem.getPose()))));

    m_driverController.a().whileTrue(driveSubsystem.bumpRotation( () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX()));

    m_driverController.b().whileTrue(driveSubsystem.brake());
  
    m_driverController.y().whileTrue(driveSubsystem.driveV(() -> m_driverController.getLeftY() * -0.5,
                                                                () -> m_driverController.getLeftX() * -0.5,
                                                                () -> m_driverController.getRightX() * 0.5)
    );                                            
    //operator bindings
    m_operatorController.pov(180).onTrue(intakePivotSubsystem.pivotDown());    
    m_operatorController.pov(0).onTrue(intakePivotSubsystem.pivotUp());                                                      
  
    m_operatorController.leftTrigger().whileTrue(intakeSubsystem.intake());
    intakeSubsystem.setDefaultCommand(intakeSubsystem.stop());

    shooterSubsystem.setDefaultCommand(shooterSubsystem.defaultCommand());
    indexerSubsystem.setDefaultCommand(Commands.run(()->{indexerSubsystem.push();}, indexerSubsystem));
    m_operatorController.rightTrigger().whileTrue(indexerSubsystem.run(()->{indexerSubsystem.enable();}).alongWith(beltFloorSubsystem.run(()->{beltFloorSubsystem.enable();})));
    m_operatorController.a().whileTrue(new PassCommand(shooterSubsystem, indexerSubsystem, beltFloorSubsystem, driveSubsystem));

    new Trigger(() -> m_operatorController.getRightY() < -0.5)
      .whileTrue(Commands.run(() -> {
        offset -= 0.2; 
      }));
      

      new Trigger(() -> m_operatorController.getRightY() > 0.5)
      .whileTrue(Commands.run(() -> {
        offset += 0.2; 
      }));
      m_operatorController.rightStick().onTrue(Commands.run(()->{offset = 0;}));

      beltFloorSubsystem.setDefaultCommand(Commands.run(()->{beltFloorSubsystem.stop();}, beltFloorSubsystem));

      m_operatorController.b().whileTrue(new ShootCommand(shooterSubsystem, ()->Constants.SpeedConstants.towerSpeed));
      m_operatorController.y().whileTrue(new ShootCommand(shooterSubsystem,  ()->Constants.SpeedConstants.trenchSpeed));
      m_operatorController.x().whileTrue(new ShootCommand(shooterSubsystem,  ()->Constants.SpeedConstants.hubSpeed));
    }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
