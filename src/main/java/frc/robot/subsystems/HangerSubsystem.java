package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;

public class HangerSubsystem extends SubsystemBase{

        private final CANSparkFlex hangerMotor = new CANSparkFlex(IntakeConstants.hangerID, MotorType.kBrushless);
        private final SparkPIDController hangerPIDController = hangerMotor.getPIDController();
        private final RelativeEncoder hangerEncoder = hangerMotor.getEncoder();
        public HangerSubsystem(){
            hangerPIDController.setP(5);
            hangerPIDController.setI(0);
            hangerPIDController.setD(0);
            
            hangerMotor.setIdleMode(IdleMode.kBrake);
       }

        double setpoint = 0;
        public Command hangerUP(){
            return this.runOnce(this::hangerUp);
        }

        public Command hangerDOWN(){
            return this.runOnce(this::hangerDown);
        }

        public void hangerUp(){
            setpoint = ((((-500 * 4 /3) / 5)* 4 )/ 34)*46;
        }

        public void hangerDown(){
            setpoint = 0;
        }

        public void stopHanger(){
            hangerMotor.stopMotor();
        }

        @Override
        public void periodic(){

            hangerPIDController.setReference(setpoint, ControlType.kPosition);
            SmartDashboard.putNumber("hanger Position", hangerEncoder.getPosition());
            Logger.recordOutput("Hanger/speed", hangerEncoder.getVelocity());
        }
}
