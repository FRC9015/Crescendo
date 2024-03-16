package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    
    UsbCamera camera1 = CameraServer.startAutomaticCapture();
    UsbCamera camera2 = CameraServer.startAutomaticCapture();
    
    public CameraSubsystem(){
        camera1.setResolution(240,240);
        camera1.setFPS(24);

        camera2.setResolution(240,240);
        camera2.setFPS(24);
    }
    
}