package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    //Creates motors
    private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeConstants.kIntakeArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeTopRoller = new CANSparkMax(IntakeConstants.kIntakeTopRollerCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeBottomRoller = new CANSparkMax(IntakeConstants.kIntakeBottomRollerCanId, MotorType.kBrushless);

    private boolean isUp = true;

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget TeleopArmTab = TeleopTab
    .add("Arm is Up", isUp)
    .withWidget(BuiltInWidgets.kBooleanBox);

    public IntakeSubsystem() {

        //Factory resets all of the sparks to know the state of them
        m_IntakeArm.restoreFactoryDefaults();
        m_IntakeTopRoller.restoreFactoryDefaults();
        m_IntakeBottomRoller.restoreFactoryDefaults();

        //Sets motors Idle Modes
        m_IntakeArm.setIdleMode(IntakeConstants.kIntakeArmIdleMode);
        m_IntakeTopRoller.setIdleMode(IntakeConstants.kIntakeTopRollerIdleMode);
        m_IntakeBottomRoller.setIdleMode(IntakeConstants.kIntakeBottomRollerIdleMode);

        //Sets motors current limits
        m_IntakeArm.setSmartCurrentLimit(IntakeConstants.kIntakeArmCurrentLimit);
        m_IntakeTopRoller.setSmartCurrentLimit(IntakeConstants.kIntakeTopRollerCurrentLimit);
        m_IntakeBottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeBottomRollerCurrentLimit);

        //Sets if the motors are inverted
        m_IntakeArm.setInverted(IntakeConstants.kIntakeArmInverted);
        m_IntakeTopRoller.setInverted(IntakeConstants.kIntakeTopRollerInverted);
        m_IntakeBottomRoller.setInverted(IntakeConstants.kIntakeBottomRollerInverted);

        m_IntakeArm.burnFlash();
        m_IntakeTopRoller.burnFlash();
        m_IntakeBottomRoller.burnFlash();
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Arm Position Rotations", m_IntakeArm.getEncoder().getPosition()* 360);
        SmartDashboard.putNumber("Roller Speed RPM", m_IntakeTopRoller.getEncoder().getVelocity());
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
}
