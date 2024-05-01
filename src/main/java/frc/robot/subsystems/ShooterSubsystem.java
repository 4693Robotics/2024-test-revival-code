package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConstants.kShooterTopCanId , MotorType.kBrushless);
    private CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConstants.kShooterBottomCanId, MotorType.kBrushless);
    private CANSparkMax m_FeederRight = new CANSparkMax(ShooterConstants.kFeederRightCanId, MotorType.kBrushless);
    private CANSparkMax m_ShooterAngle = new CANSparkMax(ShooterConstants.kShooterAngleCanId, MotorType.kBrushless);

    private SparkPIDController m_ShooterAnglePID = m_ShooterAngle.getPIDController();

    private RelativeEncoder m_ShooterAngleEncoder = m_ShooterAngle.getAlternateEncoder(Type.kQuadrature, 1);

    private double shooterDisiredPosition = 0.18;

    ShuffleboardTab PreGameTab = Shuffleboard.getTab(ShuffleboardConstants.kPreGameTabName);
    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget ShooterGetPositionTab = TeleopTab
        .add("Shooter position", m_ShooterAngleEncoder.getPosition());

    SimpleWidget ShooterSetPositionWidget = TeleopTab
        .add("Shooter set Position", shooterDisiredPosition)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("max", 0.6, "min", 0));


    /**
     * This subsystem contains the shooter for shooting notes for the
     * 2024 CRESENDO FRC competition
     */
    public ShooterSubsystem() {

         //Factory resets all of the sparks to know the state of them
        m_ShooterTop.restoreFactoryDefaults();
        m_ShooterBottom.restoreFactoryDefaults();
        m_FeederRight.restoreFactoryDefaults();
        m_ShooterAngle.restoreFactoryDefaults();
        
        //Sets motors Idle Modes
        m_ShooterTop.setIdleMode(ShooterConstants.kShooterTopIdleMode);
        m_ShooterBottom.setIdleMode(ShooterConstants.kShooterBottomIdleMode);
        m_FeederRight.setIdleMode(ShooterConstants.kFeederRightIdleMode);
        m_ShooterAngle.setIdleMode(ShooterConstants.kShooterAngleIdleMode);

        //Sets motors current limits
        m_ShooterTop.setSmartCurrentLimit(ShooterConstants.kShooterTopCurrentLimit);
        m_ShooterBottom.setSmartCurrentLimit(ShooterConstants.kShooterBottomCurrentLimit);
        m_FeederRight.setSmartCurrentLimit(ShooterConstants.kFeederRightCurrentLimit);
        m_ShooterAngle.setSmartCurrentLimit(ShooterConstants.kShooterAngleCurrentLimit);

        //Sets if the motors are inverted
        m_ShooterTop.setInverted(ShooterConstants.kShooterTopInverted);
        m_ShooterBottom.setInverted(ShooterConstants.kShooterBottomInverted);
        m_FeederRight.setInverted(ShooterConstants.kFeederRightInverted);
        m_ShooterAngle.setInverted(ShooterConstants.kShooterAngleInverted);

        m_ShooterAnglePID.setP(0.2);
        m_ShooterAnglePID.setI(0.0002);
        m_ShooterAnglePID.setD(0);

        m_ShooterAnglePID.setOutputRange(-0.5, 0.28);

        m_ShooterAnglePID.setFeedbackDevice(m_ShooterAngleEncoder);
        
        //Writes all settings to the sparks
        m_ShooterTop.burnFlash();
        m_ShooterBottom.burnFlash();
        m_FeederRight.burnFlash();
        m_ShooterAngle.burnFlash();
    }

    public void periodic() {

        ShooterGetPositionTab.getEntry().setDouble(m_ShooterAngleEncoder.getPosition());
        //shooterDisiredPosition = ShooterSetPositionWidget.getEntry().getDouble(shooterDisiredPosition);

        SmartDashboard.putNumber("Shooter Top speed", m_ShooterTop.get());
        SmartDashboard.putNumber("Shooter Bottom Speed", m_ShooterBottom.get());

        m_ShooterAnglePID.setReference(shooterDisiredPosition, ControlType.kPosition);
    }

    /**
     * sets the shooter speed
     * @param speed
     */
    public void setShooterSpeed(double speed) {
        m_ShooterTop.set(speed);
        m_ShooterBottom.set(speed);
    }

    /**
     * sets the feeder wheels speed
     * @param speed
     */
    public void setFeederSpeed(double speed) {
        m_FeederRight.set(speed);
    }


    public void setShooterPosition(double setPosition) {
        shooterDisiredPosition = setPosition;
    }
}
