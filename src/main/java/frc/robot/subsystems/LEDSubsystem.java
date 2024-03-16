// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

import java.awt.Color;

import static frc.robot.RobotContainer.INTAKE;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LED. */
  private CANdle candle = new CANdle(Constants.LEDConstants.CANDleID1);

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
    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 100);
    }

    public void setColor(Color color){
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
        candle.modulateVBatOutput(0.9);
    }
    public void setSampleAnimation() {
        // create a rainbow animation:
        // - 90% brightness
        // - 1/2 speed
        // - 64 LEDs
        RainbowAnimation rainbowAnim = new RainbowAnimation(0.9, 0.5, 64);
        candle.animate(rainbowAnim);
    }

    public void indicateNote() {
        if (INTAKE.getNoteStatus()) {
            candle.configBrightnessScalar(0.9);
            setSampleAnimation();
        } else {
            candle.configBrightnessScalar(0);

        }
    }
    public void  clearLEDs(){
        candle.clearAnimation(0);
    }

}