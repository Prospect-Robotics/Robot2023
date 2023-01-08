package com.team2813.lib.imu;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team2813.lib.util.ConfigUtils;

public class PigeonWrapper extends PigeonIMU {

    public PigeonWrapper(int deviceNumber) {
        super(deviceNumber);

        ConfigUtils.ctreConfig(() -> configAllSettings(new PigeonIMUConfiguration()));
        ConfigUtils.ctreConfig(
                () -> setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20)
        );
    }

    public double getHeading() {
        return getYaw();
    }

    public void setHeading(double angle) {
        setYaw(angle);
        setAccumZAngle(0);
    }
}