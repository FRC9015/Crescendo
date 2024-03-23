// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import java.awt.Color;
import static frc.robot.RobotContainer.*;

public class LEDSubsystem extends SubsystemBase {
    /** Creates a new LED. */
    private static int NUM_LEDS = 80;
    private CANdle candle = new CANdle(Constants.LEDConstants.candleID1);
    private Animation bufferedAnimation = new RainbowAnimation(0.7, 0.2,NUM_LEDS);

    public LEDSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(candleConfiguration, 100);
        candle.clearAnimation(0);
    }

    public void setColor(Color color){
        bufferedAnimation = new TwinkleAnimation(color.getRed(), color.getGreen(), color.getBlue(),0,1,NUM_LEDS, TwinkleAnimation.TwinklePercent.Percent100);
        //candle.modulateVBatOutput(0.9);
    }
    public void setNoteAnimation() {
        // create a rainbow animation:
        // - 90% brightness
        // - 1/2 speed
        // - 64 LEDs
        RainbowAnimation rainbowAnim = new RainbowAnimation(0.9, 0.5, 64);
        candle.animate(rainbowAnim);
    }
    public void strobeAnimation(Color color){
        bufferedAnimation = new StrobeAnimation(color.getRed(), color.getGreen(), color.getBlue(),0, 0.25,NUM_LEDS);
    }

    public void indicateNote() {
        if (INTAKE.getShooterSensor() && INTAKE.getHandoffStatus()){
            setColor(Color.GREEN);
        }
        else if (INTAKE.getHandoffStatus()) {
            setColor(Color.RED);
        }
        if (INTAKE.getShooterSensor()) {
            setColor(Color.GREEN);
        }
    }

    public void updateLEDs() {
        candle.animate(bufferedAnimation);
        strobeAnimation(Color.black);
    }

    public void  clearLEDs(){
        candle.clearAnimation(0);
    }

}