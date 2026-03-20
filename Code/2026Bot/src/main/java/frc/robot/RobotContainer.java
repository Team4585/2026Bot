// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  double offset = 0;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
   private final CommandXboxController m_operatorController = 
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    SwerveInputStream driveStream = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
                                                                () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX())
                                                            .withControllerRotationAxis(()->m_driverController.getRightX() * -1)
                                                            .deadband(OperatorConstants.deadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);


  public RobotContainer() {
    configureBindings();
  }

  

  private void configureBindings() {
    //driving bindings
    SwerveInputStream autoAimStream = driveStream.copy();
    autoAimStream.aim(new Pose2d(RobotMath.getHubPosition(), Rotation2d.kZero))
      .aimWhile(m_driverController.rightTrigger())
      .scaleTranslation(0.5);;

    Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveStream);

    driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.a().whileTrue(driveSubsystem.bumpRotation( () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX()));

    m_driverController.b().whileTrue(driveSubsystem.brake());
  
    m_driverController.y().whileTrue(driveSubsystem.pointToHeading( () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX()));
                                                
    //operator bindings
    m_operatorController.pov(0).onTrue(intakePivotSubsystem.pivotDown());    
    m_operatorController.pov(180).onTrue(intakePivotSubsystem.pivotUp());                                                      
  
    m_operatorController.leftTrigger().whileTrue(intakeSubsystem.intake());
    intakeSubsystem.setDefaultCommand(intakeSubsystem.stop());

    shooterSubsystem.setDefaultCommand(shooterSubsystem.defaultCommand());
    indexerSubsystem.setDefaultCommand(Commands.run(()->{indexerSubsystem.push();}));
    m_operatorController.rightTrigger().whileTrue(new ShootCommand(shooterSubsystem, indexerSubsystem, driveSubsystem, offset));
  
    new Trigger(() -> m_driverController.getRightY() < -0.5)
      .whileTrue(Commands.run(() -> {
        offset += 0.2; 
      }));

      new Trigger(() -> m_driverController.getRightY() < 0.5)
      .whileTrue(Commands.run(() -> {
        offset -= 0.2; 
      }));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
  }
}
