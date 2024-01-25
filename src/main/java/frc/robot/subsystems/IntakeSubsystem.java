package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    //Creates motors
    private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeMiniMotor = new CANSparkMax(IntakeConstants.kIntakeMiniCanId, MotorType.kBrushless);

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    public IntakeSubsystem() {

        //Sets motors to brushless config
        m_IntakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
        m_IntakeMiniMotor.setIdleMode(IntakeConstants.kIntakeMiniIdleMode);
    }

    //Function to make intake move
    public void moveIntake(double speed) {
        m_IntakeMotor.set(speed);
    }
    
    //Function 
    public void moveIntakeMini(double speed) {
        m_IntakeMiniMotor.set(speed);
    }
}
