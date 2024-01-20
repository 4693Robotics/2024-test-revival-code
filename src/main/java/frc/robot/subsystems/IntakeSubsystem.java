package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final CANSparkMax m_IntakeMiniMotor = new CANSparkMax(IntakeConstants.kIntakeMiniCanId, MotorType.kBrushless);

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    public IntakeSubsystem() {
        m_IntakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
        m_IntakeMiniMotor.setIdleMode(IntakeConstants.kIntakeMiniIdleMode);
    }

    SimpleWidget IntakeTempWidget = TeleopTab
    .add("Intake Temp", m_IntakeMotor.getMotorTemperature())
    .withWidget(BuiltInWidgets.kDial);

    public void moveIntake(double speed) {
        m_IntakeMotor.set(speed);
    }
    
    public void moveIntakeMini(double speed) {
        m_IntakeMiniMotor.set(speed);
    }
}
