package com.team2813.lib.motors;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Wrapper class for licensed TalonFX motor controllers
 */
public class TalonFXProWrapper extends TalonFX {
    private final List<TalonFX> followers = new ArrayList<>();
    private final Map<String, StatusSignalValue<Object>> statusSignals = new HashMap<>();
    private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    private StatusSignalValue<Double> encoderPosition = getRotorPosition();
    private StatusSignalValue<Double> motorVelocity = getRotorVelocity();

    /**
     * Constructor
     * @param deviceNumber [0,62]
     * @param canbus Name of the CANbus; can be a SocketCAN interface (on Linux),
     *               or a CANivore device name or serial number
     * @param invertType Invert state of the motor
     */
    public TalonFXProWrapper(int deviceNumber, String canbus, InvertedValue invertType) {
        super(deviceNumber, canbus);

        motorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorConfiguration.Voltage.PeakReverseVoltage = -12;

        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;

        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        motorConfiguration.MotorOutput.Inverted = invertType;

        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration));
    }

    /**
     * Constructor
     * @param deviceNumber [0,62]
     * @param invertType Invert state of the motor
     */
    public TalonFXProWrapper(int deviceNumber, InvertedValue invertType) {
        super(deviceNumber);

        motorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorConfiguration.Voltage.PeakReverseVoltage = -12;

        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;

        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        motorConfiguration.MotorOutput.Inverted = invertType;

        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration));
    }

    /**
     * If control mode is duty cycle, demand should a value between -1 and 1 (fractional power)
     * If control mode is velocity, demand should be desired motor velocity in RPM
     * If control mode is motion magic, demand should be a desired motor position in rotations
     */
    public void set(ControlMode controlMode, double demand) {
        switch (controlMode) {
            case DUTY_CYCLE:
                setControl(new DutyCycleOut(demand));
                break;
            case VELOCITY:
                set(controlMode, demand, 0, false);
                break;
            case MOTION_MAGIC:
                setControl(new MotionMagicTorqueCurrentFOC(demand));
                break;
        }
    }

    public void set(ControlMode controlMode, double demand, double feedForward, boolean brakeMode) {
        switch (controlMode) {
            case VELOCITY:
                demand /= 60;
                if (feedForward == 0) setControl(new VelocityTorqueCurrentFOC(demand, feedForward, 0, brakeMode));
                else setControl(new VelocityVoltage(demand, true, feedForward, 0, brakeMode));
                break;
            case MOTION_MAGIC:
                if (feedForward == 0) setControl(new MotionMagicTorqueCurrentFOC(demand, feedForward, 0, brakeMode));
                else setControl(new MotionMagicVoltage(demand, true, feedForward, 0, brakeMode));
                break;
        }
    }

    /**
     * Sets the motor to a current, which scales to a torque
     * based on the motor's kT constant.
     * @param current current in Amps
     */
    public void setTorqueCurrent(double current) {
        setControl(new TorqueCurrentFOC(current, 1, 1, false));
    }

    public void setTorqueCurrent(double current, double maxAbsDutyCycle, boolean brakeMode) {
        setControl(new TorqueCurrentFOC(current, maxAbsDutyCycle, 1, brakeMode));
    }

    /**
     * @return motor encoder position in rotations
     */
    public double getEncoderPosition() {
        encoderPosition = encoderPosition.refresh();
        return encoderPosition.getValue();
    }

    /**
     * @param position desired position in motor rotations
     */
    public void setEncoderPosition(double position) {
        setRotorPosition(position);
    }

    /**
     * @return motor velocity in RPM
     */
    public double getMotorVelocity() {
        motorVelocity = motorVelocity.refresh();
        return motorVelocity.getValue() * 60;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        motorConfiguration.MotorOutput.NeutralMode = neutralMode;
        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.MotorOutput));
    }

    public void configPID(int slot, double kP, double kI, double kD) {
        switch (slot) {
            case 0:
                motorConfiguration.Slot0.kP = kP;
                motorConfiguration.Slot0.kI = kI;
                motorConfiguration.Slot0.kD = kD;

                ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.Slot0));
                break;
            case 1:
                motorConfiguration.Slot1.kP = kP;
                motorConfiguration.Slot1.kI = kI;
                motorConfiguration.Slot1.kD = kD;

                ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.Slot1));
                break;
            case 2:
                motorConfiguration.Slot2.kP = kP;
                motorConfiguration.Slot2.kI = kI;
                motorConfiguration.Slot2.kD = kD;

                ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.Slot2));
                break;
            default:
                DriverStation.reportError("PID Slot " + slot + " does not exist", false);
                break;
        }
    }

    public void configPID(double kP, double kI, double kD) {
        configPID(0, kP, kI, kD);
    }

    /**
     * @param maxVelocity max velocity in motor rotations per second
     * @param maxAcceleration max acceleration in motor rotations per second^2
     * @param maxJerk max jerk (derivative of acceleration) in motor rotations per second^3
     */
    public void configMotionMagic(double maxVelocity, double maxAcceleration, double maxJerk) {
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;
        motorConfiguration.MotionMagic.MotionMagicJerk = maxJerk;
        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.MotionMagic));
    }

    public void configMotionMagic(double maxVelocity, double maxAcceleration) {
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;
        ConfigUtils.ctreProConfig(() -> getConfigurator().apply(motorConfiguration.MotionMagic));
    }

    public void configPositionSignalUpdateFrequency(double frequencyHz) {
        ConfigUtils.ctreProConfig(() -> encoderPosition.setUpdateFrequency(frequencyHz));
    }

    public void configVelocitySignalUpdateFrequency(double frequencyHz) {
        ConfigUtils.ctreProConfig(() -> motorVelocity.setUpdateFrequency(frequencyHz));
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
        return statusSignal.getValue();
    }

    public void addFollower(int deviceNumber, String canbus, boolean opposeMaster) {
        TalonFX follower = new TalonFX(deviceNumber, canbus);
        follower.setControl(new Follower(this.getDeviceID(), opposeMaster));
        followers.add(follower); // add to follower list so TalonFX follower object is preserved
    }

    public void addFollower(int deviceNumber, boolean opposeMaster) {
        TalonFX follower = new TalonFX(deviceNumber);
        follower.setControl(new Follower(this.getDeviceID(), opposeMaster));
        followers.add(follower); // add to follower list so TalonFX follower object is preserved
    }
}