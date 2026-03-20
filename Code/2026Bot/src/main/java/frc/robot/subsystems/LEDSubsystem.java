package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{
    private final CANdle candle = new CANdle(Constants.CANids.candleID);
}
