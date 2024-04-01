package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

    private final PWM m_lights = new PWM(1);
    private final Timer lightTimer = new Timer();
    private boolean lightOn = true;
    private boolean lightsSolid = true;
    private double solidColor = 0.99;
    private double blinkingColor = 0.99;
    
    public LightSubsystem() {

        lightTimer.start();
    }

    public void periodic() {
        if (lightsSolid) {
            m_lights.setSpeed(solidColor);
        } else {
            if (lightOn) {
                m_lights.setSpeed(blinkingColor);
            } else {
                m_lights.setSpeed(0.99);
            }
        };

        if (lightTimer.get() > 0.15) {
            lightTimer.restart();
            lightOn = !lightOn;
        }

        SmartDashboard.putNumber("Solid", solidColor);
        SmartDashboard.putNumber("Blinking", blinkingColor);


    }

    public void setLightColor(double pwmSpeed) {
        solidColor = pwmSpeed;
        lightsSolid = true;
    }

    public void setBlinkingLightColor(double pwmSpeed) {
        blinkingColor = pwmSpeed;
        lightsSolid = false;
    }

    

}
