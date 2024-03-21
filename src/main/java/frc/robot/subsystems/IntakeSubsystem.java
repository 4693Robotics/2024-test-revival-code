package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class IntakeSubsystem extends SubsystemBase {

    //Creates motors
    private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeConstants.kIntakeArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeTopRoller = new CANSparkMax(IntakeConstants.kIntakeTopRollerCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeBottomRoller = new CANSparkMax(IntakeConstants.kIntakeBottomRollerCanId, MotorType.kBrushless);

    private final AbsoluteEncoder m_IntakeArmEncoder = m_IntakeArm.getAbsoluteEncoder();

    private boolean isIn = true;

    ShuffleboardTab PreGameTab = Shuffleboard.getTab(ShuffleboardConstants.kPreGameTabName);
    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget TeleopArmTab = TeleopTab
        .add("Arm is in", isIn)
        .withWidget(BuiltInWidgets.kBooleanBox);

    /**
     * This subsystem contains the intake for picking up notes for the
     * 2024 CRESENDO FRC competition
     */
    public IntakeSubsystem() {
        System.out.println();

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

        //Writes all settings to the sparks
        m_IntakeArm.burnFlash();
        m_IntakeTopRoller.burnFlash();
        m_IntakeBottomRoller.burnFlash();
    }

    public void periodic() {
        TeleopArmTab.getEntry().setBoolean(isIn);
    }

    //Function to make intake arm move
    public void setIntakeArmSpeed(double speed) {
        m_IntakeArm.set(speed);
    }
    
    //Function to make intake roller move
    public void setIntakeRollerSpeed(double speed) {
        m_IntakeTopRoller.set(Math.min(0.5, speed));
        m_IntakeBottomRoller.set(Math.min(0.5, speed));
    }

    public void setIsIntakeUp(boolean value) {
        isIn = value;
    }

    public double getArmPosition() {
        return m_IntakeArmEncoder.getPosition();
    }
}
