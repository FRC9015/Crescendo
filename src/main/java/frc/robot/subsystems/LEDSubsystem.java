// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDS. */

  CANdle candle = new CANdle(Constants.LEDConstants.CANDleID1);

    //for reference; actually type in the numbers into the setLEDs method when giving the Leds a color.
    private static final Color orange = new Color(255, 25, 0);
    private static final Color purple = new Color(184, 0, 185);
    private static final Color yellow = new Color(242, 60, 0);
    private static final Color green = new Color(56, 209, 0);
    private static final Color blue = new Color(8, 32, 255);
    private static final Color red = new Color(255, 0, 0);
    private static final Color white = new Color(255, 255, 255);
    public LEDSubsystem() {
            CANdleConfiguration candleConfiguration = new CANdleConfiguration();
            candleConfiguration.statusLedOffWhenActive = true;
            candleConfiguration.disableWhenLOS = false;
            candleConfiguration.stripType = LEDStripType.RGB;
            candleConfiguration.brightnessScalar = 1.0;
            candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
            candle.configAllSettings(candleConfiguration, 100);

            setDefaultCommand(new InstantCommand());

    }
    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 100);
    }

    public void setOrange(){
        candle.setLEDs(orange.red, orange.green, orange.blue);
        candle.modulateVBatOutput(0.9);
    }
    public void setBlue(){
        candle.setLEDs(blue.red,blue.green,blue.blue);
        candle.modulateVBatOutput(0.9);
    }
    public void setGreen(){
        candle.setLEDs(green.red,green.green,green.blue);
        candle.modulateVBatOutput(0.9);
    }
    public void setPurple(){
        candle.setLEDs(purple.red, purple.green, purple.blue);
        candle.modulateVBatOutput(0.9);
    }
    public void setYellow(){
        candle.setLEDs(yellow.red, yellow.green, yellow.blue);
        candle.modulateVBatOutput(0.9);
    }
    public void setRed(){
        candle.setLEDs(red.red,red.green,red.blue);
        candle.modulateVBatOutput(0.9);
    }

    public void setWhite(){
        candle.setLEDs(white.red, white.green, white.blue);
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


    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}