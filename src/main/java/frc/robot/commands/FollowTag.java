
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.*;
import frc.robot.subsystems.LimelightInterface;





public class FollowTag extends Command{
   
    private LimelightInterface limelight = new LimelightInterface();
    
	public Transform2d target;
	public double fvx, fvy;
	
	double rotationalVelocity = -driveController.getRightX();
	
    private double x = limelight.getX();
    private double y = limelight.getY();
    private double theta = 330- y *(PI / 180);
    
    final PIDController xPID = new PIDController(4, 0, 1.5);
	final PIDController yPID = new PIDController(4, 0, 1.5);
	final PIDController wPID = new PIDController(4, 0, 4);
    
	
   
    @Override
	public void execute() {
		 SWERVE.drive(new ChassisSpeeds(1+x,1+y,1-rotationalVelocity));
	}	
}
