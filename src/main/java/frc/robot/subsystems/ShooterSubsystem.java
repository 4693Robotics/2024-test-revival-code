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

    /**
     * This subsystem contains the shooter for shooting notes for the
     * 2024 CRESENDO FRC competition
     */
    public ShooterSubsystem() {

         //Factory resets all of the sparks to know the state of them
        m_ShooterTop.restoreFactoryDefaults();
        m_ShooterBottom.restoreFactoryDefaults();
        m_FeederRight.restoreFactoryDefaults();
        m_FeederLeft.restoreFactoryDefaults();
        
        //Sets motors Idle Modes
        m_ShooterTop.setIdleMode(ShooterConstants.kShooterTopIdleMode);
        m_ShooterBottom.setIdleMode(ShooterConstants.kShooterBottomIdleMode);
        m_FeederRight.setIdleMode(ShooterConstants.kFeederRightIdleMode);
        m_FeederLeft.setIdleMode(ShooterConstants.kFeederLeftIdleMode);

        //Sets motors current limits
        m_ShooterTop.setSmartCurrentLimit(ShooterConstants.kShooterTopCurrentLimit);
        m_ShooterBottom.setSmartCurrentLimit(ShooterConstants.kShooterBottomCurrentLimit);
        m_FeederRight.setSmartCurrentLimit(ShooterConstants.kFeederRightCurrentLimit);
        m_FeederLeft.setSmartCurrentLimit(ShooterConstants.kFeederLeftCurrentLimit);

        //Sets if the motors are inverted
        m_ShooterTop.setInverted(ShooterConstants.kShooterTopInverted);
        m_ShooterBottom.setInverted(ShooterConstants.kShooterBottomInverted);
        m_FeederRight.setInverted(ShooterConstants.kFeederRightInverted);
        m_FeederLeft.setInverted(ShooterConstants.kFeederLeftInverted);

        //Writes all settings to the sparks
        m_ShooterTop.burnFlash();
        m_ShooterBottom.burnFlash();
        m_FeederRight.burnFlash();
        m_FeederLeft.burnFlash();
    }

    public void periodic() {
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
}
