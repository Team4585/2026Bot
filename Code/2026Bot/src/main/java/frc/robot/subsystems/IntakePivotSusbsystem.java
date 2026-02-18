package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivotSusbsystem extends SubsystemBase{
    SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.PIDFFControllers.intakePivotPID.kP,Constants.PIDFFControllers.intakePivotPID.kI, Constants.PIDFFControllers.intakePivotPID.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(Constants.PIDFFControllers.intakePivotFF)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(60)))
        .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

        private SparkMax sparkMax = new SparkMax(Constants.CANids.intakePivotMotorID, MotorType.kBrushless);

        private SmartMotorController motorController = new SparkWrapper(sparkMax, DCMotor.getNeoVortex(1), pivotMotorConfig);

        private ArmConfig intakePivotConfig = new ArmConfig(motorController)
        .withSoftLimits(Degrees.of(0), Degrees.of(90))
        .withHardLimit(Degrees.of(-10), Degrees.of(95))
        .withStartingPosition(Degrees.of(0))
        .withLength(Feet.of(0.83333))
        .withMass(Pounds.of(10))
        .withTelemetry("Intake Pivot", TelemetryVerbosity.HIGH);

        private Arm intakePivot= new Arm(intakePivotConfig);

        public void setAngleSetpoint(Angle angle) { 
            intakePivot.setMechanismPositionSetpoint(angle); 
        }

        @Override
        public void periodic(){
            intakePivot.updateTelemetry();
        }

        @Override
        public void simulationPeriodic() {
            intakePivot.simIterate();
        }
}
