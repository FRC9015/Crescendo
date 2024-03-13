package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    
    UsbCamera camera = CameraServer.startAutomaticCapture();
    
    public CameraSubsystem(){
        camera.setResolution(640,480);
        camera.setFPS(24);
    }
    
}