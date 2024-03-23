package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;

public class HangerSubsystem extends SubsystemBase{

        private final CANSparkFlex hangerMotor = new CANSparkFlex(IntakeConstants.hangerID, MotorType.kBrushless);

        public Command hangerUP(){
            return this.startEnd(
                this::hangerUp,
                this::stopHanger);
        }

        public Command hangerDOWN(){
            return this.startEnd(
                this::hangerDown,
                this::stopHanger);
        }

        public void hangerUp(){
            hangerMotor.set(0.5);
        }

        public void hangerDown(){
            hangerMotor.set(-0.5);
        }

        public void stopHanger(){
            hangerMotor.stopMotor();
        }
}
