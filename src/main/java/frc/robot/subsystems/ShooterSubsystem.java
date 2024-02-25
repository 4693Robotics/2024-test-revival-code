package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_ShooterRight = new CANSparkMax(ShooterConstants.kShooterRightCanId , MotorType.kBrushless);
    private CANSparkMax m_ShooterLeft = new CANSparkMax(ShooterConstants.kSchooterLeftCanId, MotorType.kBrushless);

    public ShooterSubsystem() {
        
        m_ShooterRight.setIdleMode(ShooterConstants.kShooterRightIdleMode);
        m_ShooterLeft.setIdleMode(ShooterConstants.kShooterLeftIdleMode);

        m_ShooterRight.setInverted(ShooterConstants.kShooterRightInverted);
        m_ShooterLeft.setInverted(ShooterConstants.kShooterLeftInverted);
    }

    public void periodic() {
        SmartDashboard.putNumber("Right RPM", m_ShooterRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("Left RPM", m_ShooterLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Right power", m_ShooterLeft.get());
        SmartDashboard.putNumber("Left power", m_ShooterLeft.get());
    }

    public void setShooterSpeed(double speed) {

        m_ShooterRight.set(speed);
        m_ShooterLeft.set(speed);
    }
}
