package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.RobotMath;
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
    private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.PIDFFControllers.shooterPID.kP, Constants.PIDFFControllers.shooterPID.kI, Constants.PIDFFControllers.shooterPID.kD)
        .withFeedforward(Constants.PIDFFControllers.shooterFF)
        .withSimClosedLoopController(0.1, 0, 0.01)
        .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
        .withFollowers(Pair.of(new SparkMax(Constants.CANids.shooterMotor2ID, MotorType.kBrushless), true))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(0.5)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(80))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    private SmartMotorController sparkMax = new SparkWrapper(motor1, DCMotor.getNeoVortex(2), motorConfig);
  
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkMax)
    .withDiameter(Inches.of(4))
    .withUpperSoftLimit(RPM.of(8500))
    .withMass(Pounds.of(Constants.shooterAxleWeight/4 + Constants.shooterUpductedWeight));
    private FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem(){

    }

    public Command setVelocityCommand(double speed){
      return runOnce(()->{shooter.setMechanismVelocitySetpoint(RPM.of(speed));});
    }

    public void setVelocity(double speed){
      shooter.setMechanismVelocitySetpoint(RPM.of(speed));
    }

    public boolean ready(){
      return (shooter.getSpeed().magnitude()*60 >= 
       (shooter.getMechanismSetpointVelocity().get().magnitude() - Constants.shooterReadyThreshold));
    }

    public Command defaultCommand(){
      return Commands.run(()->{
        if(RobotMath.activeLater() || RobotMath.hubActive()){
          shooter.setMechanismVelocitySetpoint(RPM.of(1500));
        }
        else{
          shooter.setMechanismVelocitySetpoint(RPM.of(100));
        }
      }, this);
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
