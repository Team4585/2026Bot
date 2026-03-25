package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMath;

public class LEDSubsystem extends SubsystemBase{
    private final CANdle candle = new CANdle(Constants.CANids.candleID);

    CANdleConfiguration config = new CANdleConfiguration();
    LEDConfigs ledConfig = new LEDConfigs();
    
    private final StrobeAnimation disconnectBlink = new StrobeAnimation(0, 127);
    private final SolidColor disabledGreen = new SolidColor(0, 127).withColor(new RGBWColor(0, 255, 0));
    
    private final StrobeAnimation policeRed = new StrobeAnimation(0, 67);
    private final StrobeAnimation policeBlue = new StrobeAnimation(67, 127);

    private final FireAnimation fireAnimation = new FireAnimation(0, 127);

    private final ColorFlowAnimation hubActiveAnimation = new ColorFlowAnimation(0, 127);
    private final SolidColor hubInactiveAnimation = new SolidColor(0, 127).withColor(new RGBWColor(50, 50, 50));;
    
    public LEDSubsystem(){
        ledConfig.StripType = StripTypeValue.GRB;
        ledConfig.BrightnessScalar = 0.5;

        candle.getConfigurator().apply(config.withLED(ledConfig));
        
        disconnectBlink.withColor(new RGBWColor(255, 0, 0));
        disconnectBlink.withFrameRate(10);

        policeRed.withColor(new RGBWColor(255, 0, 0, 0))
             .withFrameRate(15); 

        policeBlue.withColor(new RGBWColor(0, 0, 255, 0))
              .withFrameRate(15);

        hubActiveAnimation.withColor(new RGBWColor(0, 255, 0));
    }

    public Command policeLights(){
        return run(()->{
           if ((edu.wpi.first.wpilibj.Timer.getFPGATimestamp() % 1.0) < 0.5) {
        candle.setControl(policeRed.withSlot(0));
        } 
        else {
        candle.setControl(policeBlue.withSlot(1));
    }
        });
    }

    public Command fire(){
        return run(()->{
            candle.setControl(fireAnimation);
        });
    }

    public Command defaultAnimation(){
        return Commands.run(()->{
            boolean hubActive = RobotMath.hubActive();
            if(hubActive){
                candle.setControl(hubActiveAnimation);
            }
            else{
                candle.setControl(hubInactiveAnimation);
            }
        }, this);
    }

    @Override
    public void periodic(){
        if(getCurrentCommand() == null){
        if(!DriverStation.isDSAttached()){
            candle.setControl(disconnectBlink);
        }
        else if(DriverStation.isDisabled()){
            candle.setControl(disabledGreen);
        }
    }
    }
}
