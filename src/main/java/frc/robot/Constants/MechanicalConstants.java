package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class MechanicalConstants {
    
    public static class DriveTrainConstants{

    
        public static final Constraints MID_CONSTRAINTS = new Constraints(1.5, 0.3);
    	public static final Constraints FWD_CONSTRAINTS = new Constraints(4, 4);
        public static final double RADIUS = 0.50833;
    }
}
