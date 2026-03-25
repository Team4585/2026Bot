package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivotSubsystem extends SubsystemBase{
  private Angle encoderOffset = Degrees.of(Constants.OffsetConstants.intakePivotEncoderOffset);
  private SparkMax sparkMax = new SparkMax(Constants.CANids.intakePivotMotorID, MotorType.kBrushless);
  private SparkAbsoluteEncoder encoder = sparkMax.getAbsoluteEncoder();


  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(Constants.PIDFFControllers.intakePivotPID.kP, Constants.PIDFFControllers.intakePivotPID.kI, Constants.PIDFFControllers.intakePivotPID.kD)
      .withFeedforward(Constants.PIDFFControllers.intakePivotFF)
      .withSimClosedLoopController(10, 0, 0)
      .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(60)
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20))
      .withExternalEncoder(encoder)
      .withExternalEncoderZeroOffset(encoderOffset) 
      .withUseExternalFeedbackEncoder(true);

  private SmartMotorController motorController = new SparkWrapper(sparkMax, DCMotor.getNEO(1), motorConfig);


  private ArmConfig pivotConfig = new ArmConfig(motorController)
      .withHardLimit(Degrees.of(55), Degrees.of(350))
      .withSoftLimits(Degrees.of(56), Degrees.of(349))
      .withLength(Feet.of(0.8333))
      .withMass(Pounds.of(10))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);


  private Arm intakePivot = new Arm(pivotConfig);

  public IntakePivotSubsystem(){
    motorController.setPosition(Constants.SetpointConstants.IntakePivotSetpoints.UpPos);
  }

  public Command pivotUp(){
    return runOnce(() -> motorController.setPosition(Constants.SetpointConstants.IntakePivotSetpoints.UpPos));
  }

  public Command pivotDown(){
    return runOnce(() -> motorController.setPosition(Constants.SetpointConstants.IntakePivotSetpoints.DownPos));
  }

  @Override
  public void periodic(){
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic(){
    intakePivot.simIterate();
  }
}