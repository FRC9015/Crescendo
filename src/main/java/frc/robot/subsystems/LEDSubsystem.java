// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates new LEDs. */
private final AddressableLED lED = new AddressableLED(0);//port number needs to be changed
private final AddressableLEDBuffer lEDBuffer = new AddressableLEDBuffer(1);//port number needs to be changed


  public LEDSubsystem() 
  {
    
    lED.setLength(lEDBuffer.getLength());
    lED.setData(lEDBuffer);
    lED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

  }
  public void startLEDS(){
    lED.start();
  }
  public void endLEDS(){
    lED.stop();
  }
  
  public void SetLEDsPurple()
  { 
    System.out.println("setting to purple");
    for(int i = 0; i < lEDBuffer.getLength(); i++)
    {
      lEDBuffer.setLED(i, Color.kPurple);
    }
    System.out.println("done");


  }

  public void SetLEDsRed()
  {
    System.out.println("setting to red");
    for(int i = 0; i < lEDBuffer.getLength(); i++)
    {
      lEDBuffer.setRGB(i, 255, 0, 0);
    }
    System.out.println("done");
  }

}