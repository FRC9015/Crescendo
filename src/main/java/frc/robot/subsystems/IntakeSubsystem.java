package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    public static final SwerveSubsystem swerveDrive = new SwerveSubsystem();

	public static final Pigeon gyro = new Pigeon();

	private final LimelightInterface limelight = new LimelightInterface();

    public boolean isReadytoIntake() {
        //this does nothing, needs to be changed.
        return false;
    }

    public boolean isReadyToHandoff() {
        //this does nothing, needs to be changed.
        return false;
    }


    public Command intakeNote(){
        //this does nothing, needs to be changed.
        return new InstantCommand();
    }

    public Command returnToCollectionPosition(){
        //this does nothing, needs to be changed.
        return new InstantCommand();
    }

    public Command returnToHandoffPosition() {
        //this does nothing, needs to be changed.
        return new InstantCommand();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    }


