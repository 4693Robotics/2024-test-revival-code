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
    private final CANSparkMax m_IntakeRoller = new CANSparkMax(IntakeConstants.kIntakeRollerCanId, MotorType.kBrushless);

    private final DigitalInput m_IntakeLimit = new DigitalInput(0);

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    public IntakeSubsystem() {
        //Sets motors to brushless config
        m_IntakeArm.setIdleMode(IntakeConstants.kIntakeArmIdleMode);
        m_IntakeRoller.setIdleMode(IntakeConstants.kIntakeRollerIdleMode);

        //Sets motors currentlimit
        m_IntakeArm.setSmartCurrentLimit(IntakeConstants.kIntakeArmCurrentLimit);
        m_IntakeRoller.setSmartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);

        m_IntakeRoller.setInverted(true);
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Arm Position Rotations", m_IntakeArm.getEncoder().getPosition()* 360);
        SmartDashboard.putNumber("Roller Speed RPM", m_IntakeRoller.getEncoder().getVelocity());
        SmartDashboard.putBoolean("limit switch", m_IntakeLimit.get());
    }

    //Function to make intake arm move
    public void moveIntakeArm(double speed) {
        m_IntakeArm.set(speed);
    }
    
    //Function to make intake roller move
    public void moveIntakeRoller(double speed) {
        m_IntakeRoller.set(speed);
    }

    public void intakeForward() {
        m_IntakeRoller.set(1);
    }

    public void intakeBackward() {
        m_IntakeRoller.set(1);
    }

    public void intakeOff() {
        m_IntakeRoller.set(0);
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
