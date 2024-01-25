package frc.utils;

public class JoystickUtils {
    
    public class JoystickInputScale {

        private static Boolean isNeg(double Joystick) {
            if (Joystick < 0) {
                return true;
            } else {
                return false;
            } 
        }

        public static double applyNicethings(double stick) {
            boolean isNegitive = isNeg(stick);
            if (isNegitive) {
                return -stick;
            } else {
                return stick;
            }
        }
    }
}
