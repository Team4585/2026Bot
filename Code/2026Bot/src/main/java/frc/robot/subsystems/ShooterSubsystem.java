package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase{
    public SmartMotorController motor1;
    public BangBangController bangbang; 
    private double targetRPM = 0;

    public double bangbangthreshold = 100;
    
    public ShooterSubsystem(){
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
        .withClosedLoopController(Constants.PIDControllers.shooterPID)
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(1)
        .withControlMode(yams.motorcontrollers.SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.012));

        motor1 = new SparkWrapper(new SparkMax(Constants.CANids.shooterMotor1ID, SparkLowLevel.MotorType.kBrushless), DCMotor.getNeoVortex(1), config);
        bangbang = new BangBangController(bangbangthreshold);

        SmartDashboard.putNumber("Shooter Target", 0);
    }
  
    public void periodic() {
        targetRPM = SmartDashboard.getNumber("Shooter Target", 0);
    
        AngularVelocity currentVel = motor1.getRotorVelocity();
        double currentRPM = currentVel.in(edu.wpi.first.units.Units.RPM);
        double error = targetRPM - currentRPM;

        if (targetRPM <= 0) {
            motor1.setVoltage(edu.wpi.first.units.Units.Volts.of(0)); 
        } 
   
        else if (error > bangbangthreshold) {
            double output = bangbang.calculate(currentRPM, targetRPM);
            motor1.setVoltage(edu.wpi.first.units.Units.Volts.of(output * 12.0)); 
        } 
        else {
            motor1.setVelocity(edu.wpi.first.units.Units.RPM.of(targetRPM));
        }

        SmartDashboard.putNumber("Shooter/Actual RPM", currentRPM);
    }

    @Override
    public void simulationPeriodic(){
        motor1.simIterate();
    }
}
