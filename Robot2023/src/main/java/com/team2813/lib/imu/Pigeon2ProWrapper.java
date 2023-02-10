package com.team2813.lib.imu;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.team2813.lib.util.ConfigUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public class Pigeon2ProWrapper extends Pigeon2 {

    private double currentHeading = 0;
    private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    private StatusSignalValue<Double> yaw = getYaw();
    private StatusSignalValue<Double> pitch = getPitch();

    private final Map<String, StatusSignalValue<Object>> statusSignals = new HashMap<>();
    private final Map<StatusSignalValue<Object>, Double> statusSignalUpdateFreqs = new HashMap<>();
    private final boolean canivore;

    /**
     * Constructor
     * @param deviceNumber [0,62]
     * @param canbus Name of the CANbus; can be a SocketCAN interface (on Linux),
     *               or a CANivore device name or serial number
     */
    public Pigeon2ProWrapper(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);

        canivore = true;

        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(pigeonConfig));
    }

    /**
     * Constructor
     * @param deviceNumber [0,62]
     */
    public Pigeon2ProWrapper(int deviceNumber) {
        super(deviceNumber);

        canivore = false;

        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(pigeonConfig));
        ConfigUtils.ctreProConfig(() -> yaw.setUpdateFrequency(50));
        ConfigUtils.ctreProConfig(() -> pitch.setUpdateFrequency(50));
    }

    /**
     * @return the heading of the robot in degrees
     */
    public double getHeading() {
        yaw = yaw.refresh();
        return yaw.getValue();
    }

    /**
     * @param angle the desired heading in degrees
     */
    public void setHeading(double angle) {
        setYaw(angle);

        currentHeading = angle;
    }

    /**
     * @return the pitch of the robot
     */
    public double getRobotPitch() {
        pitch = pitch.refresh();
        return pitch.getValue();
    }

    public void registerStatusSignal(String key, StatusSignalValue<Object> statusSignal) {
        statusSignals.put(key, statusSignal);
    }

    /**
     * Configures the update frequency of a registered status signal
     * @param key the key you registered the status signal with
     * @param frequencyHz the number of times you want the signal to update in a second
     */
    public void configStatusSignalUpdateFrequency(String key, double frequencyHz) {
        statusSignalUpdateFreqs.put(statusSignals.get(key), frequencyHz);

        ConfigUtils.ctreProConfig(() -> statusSignals.get(key).setUpdateFrequency(frequencyHz));
    }

    /**
     * Gets the value of the registered status signal (e.g. you registered an encoder
     * position status signal and now you want to get the current encoder position)
     * @param key the key you registered the status signal with
     * @return the current value of the registered status signal
     */
    public Object getStatusSignalValue(String key) {
        StatusSignalValue<Object> statusSignal = statusSignals.get(key);
        statusSignal = statusSignal.refresh();
        statusSignals.replace(key, statusSignal);
        return statusSignal.getValue();
    }

    /**
     * Checks if a reset has occurred and restores non-persistent settings if so.
     * Implement periodically (e.g. in a subsystem's periodic() method)
     */
    public void periodicResetCheck() {
        if (!hasResetOccurred()) {
            currentHeading = getHeading();
        }
        else {
            // Config status frame rates if not using CANivore (CAN FD sets optimal rates already)
            if (!canivore) {
                ConfigUtils.ctreProConfig(() -> yaw.setUpdateFrequency(50));
                ConfigUtils.ctreProConfig(() -> pitch.setUpdateFrequency(50));

                statusSignalUpdateFreqs.forEach(new BiConsumer<StatusSignalValue<Object>, Double>() {
                    @Override
                    public void accept(StatusSignalValue<Object> objectStatusSignalValue, Double aDouble) {
                        ConfigUtils.ctreProConfig(() -> objectStatusSignalValue.setUpdateFrequency(aDouble));
                    }
                });
            }

            setHeading(currentHeading);
        }
    }
}
