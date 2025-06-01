package frc.robot.lib.drivers;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class Phoenix6Util {
    /**
     * Checks the specified error code for issues
     *
     * @param statusCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(StatusCode statusCode, String message) {
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(message + " " + statusCode, false);
        }
    }
    
    public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
        StatusCode code = function.get();
        int tries = 0;
        while (code != StatusCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
            code = function.get();
            tries++;
        }
        if (code != StatusCode.OK) {
            DriverStation.reportError("Failed to execute phoenix pro api call after " + numTries + " attempts", false);
            return false;
        }
        return true;
    }

    /**
     * checks the specified error code and throws an exception if there are any
     * issues
     *
     * @param statusCode error code
     * @param message   message to print if error happens
     */
    public static void checkErrorWithThrow(StatusCode statusCode, String message) {
        if (statusCode != StatusCode.OK) {
            throw new RuntimeException(message + " " + statusCode);
        }
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
        return checkErrorAndRetry(function, 1);
    }
}
