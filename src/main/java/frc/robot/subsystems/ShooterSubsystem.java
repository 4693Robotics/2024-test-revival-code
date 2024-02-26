package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConstants.kShooterTopCanId , MotorType.kBrushless);
    private CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConstants.kShooterBottomCanId, MotorType.kBrushless);
    private CANSparkMax m_FeederRight = new CANSparkMax(ShooterConstants.kFeederRightCanId, MotorType.kBrushless);
    private CANSparkMax m_FeederLeft = new CANSparkMax(ShooterConstants.kFeederLeftCanId, MotorType.kBrushless);

    public ShooterSubsystem() {
        m_ShooterTop.restoreFactoryDefaults();
        m_ShooterBottom.restoreFactoryDefaults();
        m_FeederRight.restoreFactoryDefaults();
        m_FeederLeft.restoreFactoryDefaults();
        
        m_ShooterTop.setIdleMode(ShooterConstants.kShooterTopIdleMode);
        m_ShooterBottom.setIdleMode(ShooterConstants.kShooterBottomIdleMode);
        m_FeederRight.setIdleMode(ShooterConstants.kFeederRightIdleMode);
        m_FeederLeft.setIdleMode(ShooterConstants.kFeederLeftIdleMode);

        m_ShooterTop.setInverted(ShooterConstants.kShooterTopInverted);
        m_ShooterBottom.setInverted(ShooterConstants.kShooterBottomInverted);
        m_FeederRight.setInverted(ShooterConstants.kFeederRightInverted);
        m_FeederLeft.setInverted(ShooterConstants.kFeederLeftInverted);

        m_ShooterTop.setSmartCurrentLimit(ShooterConstants.kShooterTopCurrentLimit);
        m_ShooterBottom.setSmartCurrentLimit(ShooterConstants.kShooterBottomCurrentLimit);
        m_FeederRight.setSmartCurrentLimit(ShooterConstants.kFeederRightCurrentLimit);
        m_FeederLeft.setSmartCurrentLimit(ShooterConstants.kFeederLeftCurrentLimit);

        m_ShooterTop.burnFlash();
        m_ShooterBottom.burnFlash();
        m_FeederRight.burnFlash();
        m_FeederLeft.burnFlash();
    }

    public void periodic() {
        SmartDashboard.putNumber("Top RPM", m_ShooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("Bottom RPM", m_ShooterBottom.getEncoder().getVelocity());
        SmartDashboard.putNumber("Top power", m_ShooterTop.get());
        SmartDashboard.putNumber("Bottom power", m_ShooterBottom.get());
    }

    public void setShooterSpeed(double speed) {
        m_ShooterTop.set(speed);
        m_ShooterBottom.set(speed);
    }

    public void setFeederSpeed(double speed) {
        m_FeederRight.set(speed);
        m_FeederLeft.set(speed);
    }
}
