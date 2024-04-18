package frc.utils;

public class JoystickUtils {
    

    public static double joystickSlider(double joystickAxis, double currentSetPoint) {
        
        double newDouble = currentSetPoint + joystickAxis * 0.01;

        if (newDouble > 1) {
            newDouble = 1;
        } else if (newDouble < -1) {
            newDouble = -1;
        }
        return newDouble;
    }

    public static double joystickSlider(double joystickAxis, double currentSetPoint, double maxOutput, double minOutput) {
   
        double newDouble = currentSetPoint + joystickAxis * 0.1;

        if (newDouble > maxOutput) {
            newDouble = maxOutput;
        } else if (newDouble < minOutput) {
            newDouble = minOutput;
        }
        return newDouble;
    }
}

