package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

    private final PWM m_lights = new PWM(1);
    private final Timer lightTimer = new Timer();
    private boolean lightOn = true;
    
    public LightSubsystem() {

        lightTimer.start();
    }

    public void periodic() {

        if (lightTimer.get() > 0.5) {
            lightTimer.restart();
            lightOn = !lightOn;
        }
    }

    public void setLightColor(double pwmSpeed) {
        m_lights.setSpeed(pwmSpeed);
    }

    public void setBlinkingLightColor(double pwmSpeed) {
        if (lightOn) {
            m_lights.setSpeed(pwmSpeed);
        } else {
            m_lights.setSpeed(0.99);
        }
    }

    

}
