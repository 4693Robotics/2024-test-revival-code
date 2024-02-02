package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_ShooterRight = new CANSparkMax(ShooterConstants.kShooterRightCanId , MotorType.kBrushless);
    private CANSparkMax m_ShooterLeft = new CANSparkMax(ShooterConstants.kSchooterLeftCanId, MotorType.kBrushless);
}
