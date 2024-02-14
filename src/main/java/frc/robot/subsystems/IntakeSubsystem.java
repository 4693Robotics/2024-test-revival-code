package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Creates motors
    private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeConstants.kIntakeArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeTopRoller = new CANSparkMax(IntakeConstants.kIntakeTopRollerCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeBottomRoller = new CANSparkMax(IntakeConstants.kIntakeBottomRollerCanId, MotorType.kBrushless);

    private final DigitalInput m_IntakeLimit = new DigitalInput(0);

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    public IntakeSubsystem() {
        //Sets motors to brushless config
        m_IntakeArm.setIdleMode(IntakeConstants.kIntakeArmIdleMode);
        m_IntakeTopRoller.setIdleMode(IntakeConstants.kIntakeTopRollerIdleMode);
        m_IntakeBottomRoller.setIdleMode(IntakeConstants.kIntakeBottomRollerIdleMode);

        //Sets motors currentlimit
        m_IntakeArm.setSmartCurrentLimit(IntakeConstants.kIntakeArmCurrentLimit);
        m_IntakeTopRoller.setSmartCurrentLimit(IntakeConstants.kIntakeTopRollerCurrentLimit);
        m_IntakeBottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeBottomRollerCurrentLimit);

        m_IntakeTopRoller.setInverted(true);
        m_IntakeBottomRoller.setInverted(true);
    }

    public void periodic() {
        if (this.getAtLimit()){
            setArmPosition(0);
        }
        SmartDashboard.putNumber("Intake Arm Position Rotations", m_IntakeArm.getEncoder().getPosition()* 360);
        SmartDashboard.putNumber("Roller Speed RPM", m_IntakeTopRoller.getEncoder().getVelocity());
        SmartDashboard.putBoolean("limit switch", m_IntakeLimit.get());
    }

    //Function to make intake arm move
    public void moveIntakeArm(double speed) {
        m_IntakeArm.set(speed);
    }
    
    //Function to make intake roller move
    public void moveIntakeRoller(double speed) {
        
        m_IntakeTopRoller.set(speed);
        m_IntakeBottomRoller.set(speed);
    }

    public void intakeForward() {
        m_IntakeTopRoller.set(1);
    }

    public void intakeBackward() {
        m_IntakeTopRoller.set(1);
    }

    public void intakeOff() {
        m_IntakeTopRoller.set(0);
    }

    public double getArmPosition() {
        return m_IntakeArm.getEncoder().getPosition();
    }

    public void setArmPosition(double Position) {
        m_IntakeArm.getEncoder().setPosition(Position);
    }

    public boolean getAtLimit() {
        return m_IntakeLimit.get();
    }
}
