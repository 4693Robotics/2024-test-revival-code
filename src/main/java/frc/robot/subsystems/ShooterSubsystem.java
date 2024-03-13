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
    private CANSparkMax m_ShooterAngle = new CANSparkMax(ShooterConstants.kShooterAngleCanId, MotorType.kBrushless);

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

        //Writes all settings to the sparks
        m_ShooterTop.burnFlash();
        m_ShooterBottom.burnFlash();
        m_FeederRight.burnFlash();
        m_ShooterAngle.burnFlash();
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

    /**
     * sets the shooter angle speed
     * @param speed
     */
    public void setShooterAngleSpeed(double speed) {
        m_ShooterAngle.set(speed);
    }
}
