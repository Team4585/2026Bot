// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
 // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final CommandXboxController m_operatorController = 
  //    new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    SwerveInputStream driveStream = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.deadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);


  public RobotContainer() {
    configureBindings();
  }

  

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveStream);

    driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.a().whileTrue(driveSubsystem.bumpRotation(() -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1));

    m_driverController.b().whileTrue(driveSubsystem.brake());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
  }
}
