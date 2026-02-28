// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final CommandXboxController m_operatorController = 
  //     new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

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
    Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveStream);

    driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.a().whileTrue(driveSubsystem.bumpRotation( () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX()));

    m_driverController.b().whileTrue(driveSubsystem.brake());
  
    m_driverController.y().whileTrue(driveSubsystem.pointToHeading( () -> 0.75 * m_driverController.getLeftY() * m_driverController.getLeftY() * m_driverController.getLeftY() + 0.25 * m_driverController.getLeftY(),
                                                                () -> 0.75 * m_driverController.getLeftX() * m_driverController.getLeftX() * m_driverController.getLeftX() + 0.25 * m_driverController.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
  }
}
