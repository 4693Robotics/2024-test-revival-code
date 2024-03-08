package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;

public class HangerSubsystem extends SubsystemBase {

    private CANSparkMax m_RightHanger = new CANSparkMax(HangerConstants.kRightHangerCanId, MotorType.kBrushless);
    private CANSparkMax m_LeftHanger = new CANSparkMax(HangerConstants.kLeftHangerCanId, MotorType.kBrushless);

    private RelativeEncoder m_RightHangerEncoder = m_RightHanger.getEncoder();
    private RelativeEncoder m_LeftHangerEncoder = m_LeftHanger.getEncoder();
    
    /**
     * This subsystem contains hangers for hanging the robot from a chain in
     * 2024 CRESENDO FRC competition
     */
    public HangerSubsystem() {

        //Factory resets all of the sparks to know the state of them
        m_RightHanger.restoreFactoryDefaults();
        m_LeftHanger.restoreFactoryDefaults();

        //Sets motors Idle Modes
        m_RightHanger.setIdleMode(HangerConstants.kRightHangerIdleMode);
        m_LeftHanger.setIdleMode(HangerConstants.kLeftHangerIdleMode);

        //Sets motors current limits
        m_RightHanger.setSmartCurrentLimit(HangerConstants.kRightHangerCurrentLimit);
        m_LeftHanger.setSmartCurrentLimit(HangerConstants.kLeftHangerCurrentLimit);

        //Sets if the motors are inverted
        m_RightHanger.setInverted(HangerConstants.kRightHangerInverted);
        m_LeftHanger.setInverted(HangerConstants.kLeftHangerInverted);
        
        //Clears motor faults
        m_RightHanger.clearFaults();
        m_LeftHanger.clearFaults();

        //Writes all settings to the sparks
        m_RightHanger.burnFlash();
        m_LeftHanger.burnFlash();

        //Resets all of the encoders on boot
        m_LeftHangerEncoder.setPosition(0);
        m_RightHangerEncoder.setPosition(0);
    }

    /**
     * Moves all of the hangers at full speed
     */
    public void moveHangerDown() {
        m_RightHanger.set(1);
        m_LeftHanger.set(1);
    }

    /**
     * Sets the speed of the motors
     * @param speed
     */
    public void setHangerSpeed(double speed) {
        m_RightHanger.set(speed);
        m_LeftHanger.set(speed);
    }

    /**
     * Sets the speed of the right hanger
     * @param speed
     */
    public void setRightHangerSpeed(double speed) {
        m_RightHanger.set(speed);
    }

    /**
     * Sets the speed of the left hanger
     * @param speed
     */
    public void setLeftHangerSpeed(double speed) {
        m_LeftHanger.set(speed);
    }

    /**
     * Sets the speed of the hangers to 0
     */
    public void stopHanger() {
        m_RightHanger.set(0);
        m_LeftHanger.set(0);
    }
}
