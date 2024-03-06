package frc.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShuffleboardUtils {

    /**
     * @param deviceID
     * @return true if the Spark Max is connected, false otherwise
     */
    public static boolean isSparkMaxConnected(int deviceID) {
        try {
            new CANSparkMax(deviceID, MotorType.kBrushless).close();
            return true;
        } catch (Exception e) {
            return false;
        }
    }
}
