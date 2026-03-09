package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase{
    private SparkMax motor1 = new SparkMax(Constants.CANids.shooterMotor1ID, MotorType.kBrushless);
    private SparkMax motor2 = new SparkMax(Constants.CANids.shooterMotor2ID, MotorType.kBrushless);
    private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.PIDFFControllers.shooterPID.kP, Constants.PIDFFControllers.shooterPID.kI, Constants.PIDFFControllers.shooterPID.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(Constants.PIDFFControllers.shooterFF)
        .withFollowers(Pair.of(motor2, true))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(0.5)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(80))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    private SmartMotorController sparkMax = new SparkWrapper(motor1, DCMotor.getNeoVortex(1), motorConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkMax);
    private FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem(){
        
    }

    @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
