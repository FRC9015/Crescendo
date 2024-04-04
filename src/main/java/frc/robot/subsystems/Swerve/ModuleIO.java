package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs{
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec =  0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setDriveVoltage(double volts) {}

    default void setTurnVoltage(double volts) {}

    default void setDriveBrakeMode(boolean enable) {}

    default void setTurnBrakeMode(boolean enable) {}
}
