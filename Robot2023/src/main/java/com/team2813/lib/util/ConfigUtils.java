package com.team2813.lib.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenixpro.StatusCode;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class ConfigUtils {

    private static final int ATTEMPTS = 10;

    public static void ctreConfig(Supplier<ErrorCode> configMethod) {
        ErrorCode errorCode = configMethod.get();
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(String.format("%s: %s", "Config Attempt 1 Failed", errorCode.toString()), false);
            for (int i = 1; i < ATTEMPTS; i++) {
                errorCode = configMethod.get();
                if (errorCode == ErrorCode.OK) {
                    DriverStation.reportWarning("Config Success!", false);
                    return;
                }
                else {
                    DriverStation.reportError(
                            String.format("%s: %s", "Config Attempt " + (i + 1) + " Failed", errorCode.toString()),
                            false
                    );
                }
            }
            DriverStation.reportError(String.format("%s: %s", "Config Failed", errorCode.toString()), false);
        }
    }

    public static void ctreProConfig(Supplier<StatusCode> configMethod) {
        StatusCode statusCode = configMethod.get();
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(String.format("%s: %s", "Config Attempt 1 Failed", statusCode.toString()), false);
            for (int i = 1; i < ATTEMPTS; i++) {
                statusCode = configMethod.get();
                if (statusCode == StatusCode.OK) {
                    DriverStation.reportWarning("Config Success!", false);
                    return;
                }
                else {
                    DriverStation.reportError(
                            String.format("%s: %s", "Config Attempt " + (i + 1) + " Failed", statusCode.toString()),
                            false
                    );
                }
            }
            DriverStation.reportError(String.format("%s: %s", "Config Failed", statusCode.toString()), false);
        }
    }

    public static void revConfig(Supplier<REVLibError> configMethod) {
        REVLibError error = configMethod.get();
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", "Config Attempt 1 Failed", error.toString()), false);
            for (int i = 1; i < ATTEMPTS; i++) {
                error = configMethod.get();
                if (error == REVLibError.kOk) {
                    DriverStation.reportWarning("Config Success!", false);
                    return;
                }
                else {
                    DriverStation.reportError(
                            String.format("%s: %s", "Config Attempt " + (i + 1) + " Failed", error.toString()),
                            false
                    );
                }
            }
            DriverStation.reportError(String.format("%s: %s", "Config Failed", error.toString()), false);
        }
    }
}

