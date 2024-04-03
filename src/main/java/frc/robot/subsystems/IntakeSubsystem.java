package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

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

    private final SparkPIDController m_ArmPIDController = m_IntakeArm.getPIDController();

    private final RelativeEncoder m_IntakeArmEncoder = m_IntakeArm.getAlternateEncoder(Type.kQuadrature, 1);

    private double armDisiredPosition = 0; 

    private boolean isIn = true;

    ShuffleboardTab PreGameTab = Shuffleboard.getTab(ShuffleboardConstants.kPreGameTabName);
    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget TeleopArmTab = TeleopTab
        .add("Arm is in", isIn)
        .withWidget(BuiltInWidgets.kBooleanBox);

    SimpleWidget TeleopIntakePositionTab = TeleopTab
        .add("Arm Position", m_IntakeArmEncoder.getPosition());

    SimpleWidget TeleopArmPositionTab = TeleopTab
        .add("Arm set Position", armDisiredPosition);

    /**
     * This subsystem contains the intake for picking up notes for the
     * 2024 CRESENDO FRC competition
     */
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

        m_ArmPIDController.setP(0.2);
        m_ArmPIDController.setI(0);
        m_ArmPIDController.setD(0);

        m_ArmPIDController.setFeedbackDevice(m_IntakeArmEncoder);

        //Writes all settings to the sparks
        m_IntakeArm.burnFlash();
        m_IntakeTopRoller.burnFlash();
        m_IntakeBottomRoller.burnFlash();
    }

    public void periodic() {
        TeleopArmTab.getEntry().setBoolean(isIn);
        TeleopIntakePositionTab.getEntry().setDouble(m_IntakeArmEncoder.getPosition());
        m_ArmPIDController.setReference(armDisiredPosition, ControlType.kPosition);
    }

    //Function to make intake arm move
    public void setIntakeArmSpeed(double speed) {
        m_IntakeArm.set(speed);
    }

    public void setArmPosition(double setPosition) {
        armDisiredPosition = setPosition;
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
