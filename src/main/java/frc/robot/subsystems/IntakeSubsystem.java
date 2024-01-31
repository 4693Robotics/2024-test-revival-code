package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Creates motors
    private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeRoller = new CANSparkMax(IntakeConstants.kIntakeMiniCanId, MotorType.kBrushless);

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    public IntakeSubsystem() {
        //Sets motors to brushless config
        m_IntakeArm.setIdleMode(IntakeConstants.kIntakeIdleMode);
        m_IntakeRoller.setIdleMode(IntakeConstants.kIntakeMiniIdleMode);

        //Sets motors currentlimit
        m_IntakeArm.setSmartCurrentLimit(40);
    }

    //Function to make intake move
    public void moveIntakeArm(double speed) {
        m_IntakeArm.set(speed);
    }
    
    //Function 
    public void moveIntakeRoller(double speed) {
        m_IntakeRoller.set(speed);
    }

    public void intakeOn() {
        m_IntakeRoller.set(1);
    }

    public void intakeOff() {
        m_IntakeRoller.set(0);
    }
}
