package com.team2813.lib.swerve.controllers.drive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class Falcon500DriveController implements DriveController {
    private TalonFX unlicensedMotor;
    private com.ctre.phoenixpro.hardware.TalonFX licensedMotor;
    private final double sensorPositionCoefficient;
    private final double sensorVelocityCoefficient;
    private final boolean licensed;
    private final double maxVelocity;

    private SimpleMotorFeedforward feedforward;
    private com.ctre.phoenixpro.configs.TalonFXConfiguration motorConfiguration;
    private StatusSignalValue<Double> motorPosition;
    private StatusSignalValue<Double> motorVelocity;
    private StatusSignalValue<Double> motorTemp;
    private boolean hasPidConstants = false;

    public Falcon500DriveController(int id, ModuleConfiguration moduleConfiguration, Mk4ModuleConfiguration mk4Configuration, boolean isLicensed) {
        licensed = isLicensed;
        maxVelocity = 6380.0 / 60.0 * moduleConfiguration.getDriveReduction() * moduleConfiguration.getWheelDiameter() * Math.PI;

        if (isLicensed) {
            motorConfiguration = new com.ctre.phoenixpro.configs.TalonFXConfiguration();

            sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            sensorVelocityCoefficient = sensorPositionCoefficient;

            motorConfiguration.Voltage.PeakForwardVoltage = mk4Configuration.getNominalVoltage();
            motorConfiguration.Voltage.PeakReverseVoltage = -mk4Configuration.getNominalVoltage();

            motorConfiguration.CurrentLimits.SupplyCurrentLimit = mk4Configuration.getDriveCurrentLimit();
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
            motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;

            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            licensedMotor = new com.ctre.phoenixpro.hardware.TalonFX(id);
            ConfigUtils.ctreProConfig(() -> licensedMotor.getConfigurator().apply(motorConfiguration));

            motorPosition = licensedMotor.getRotorPosition();
            ConfigUtils.ctreProConfig(() -> motorPosition.setUpdateFrequency(4, 0.25));

            motorVelocity = licensedMotor.getRotorVelocity();
            ConfigUtils.ctreProConfig(() -> motorVelocity.setUpdateFrequency(4, 0.25));

            motorTemp = licensedMotor.getDeviceTemp();
            ConfigUtils.ctreProConfig(() -> motorTemp.setUpdateFrequency(4, 0.25));
        }
        else {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / 2048;
            sensorVelocityCoefficient = sensorPositionCoefficient * 10;

            motorConfiguration.voltageCompSaturation = mk4Configuration.getNominalVoltage();

            motorConfiguration.supplyCurrLimit.currentLimit = mk4Configuration.getDriveCurrentLimit();
            motorConfiguration.supplyCurrLimit.enable = true;

            unlicensedMotor = new TalonFX(id);
            ConfigUtils.ctreConfig(() -> unlicensedMotor.configAllSettings(motorConfiguration));

            unlicensedMotor.enableVoltageCompensation(true);

            unlicensedMotor.setNeutralMode(NeutralMode.Brake);

            unlicensedMotor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            unlicensedMotor.setSensorPhase(true);

            ConfigUtils.ctreConfig(
                    () -> unlicensedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 250, 250)
            );
            ConfigUtils.ctreConfig(
                    () -> unlicensedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 125, 250)
            );
        }
    }

    public Falcon500DriveController(int id, String canbus, ModuleConfiguration moduleConfiguration, Mk4ModuleConfiguration mk4Configuration, boolean isLicensed) {
        licensed = isLicensed;
        maxVelocity = 6380.0 / 60.0 * moduleConfiguration.getDriveReduction() * moduleConfiguration.getWheelDiameter() * Math.PI;

        if (isLicensed) {
            motorConfiguration = new com.ctre.phoenixpro.configs.TalonFXConfiguration();

            sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            sensorVelocityCoefficient = sensorPositionCoefficient;

            motorConfiguration.Voltage.PeakForwardVoltage = mk4Configuration.getNominalVoltage();
            motorConfiguration.Voltage.PeakReverseVoltage = -mk4Configuration.getNominalVoltage();

            motorConfiguration.CurrentLimits.SupplyCurrentLimit = mk4Configuration.getDriveCurrentLimit();
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
            motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;

            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            licensedMotor = new com.ctre.phoenixpro.hardware.TalonFX(id, canbus);
            ConfigUtils.ctreProConfig(() -> licensedMotor.getConfigurator().apply(motorConfiguration));

            motorPosition = licensedMotor.getRotorPosition();
            motorVelocity = licensedMotor.getRotorVelocity();
            motorTemp = licensedMotor.getDeviceTemp();
        }
        else {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / 2048;
            sensorVelocityCoefficient = sensorPositionCoefficient * 10;

            motorConfiguration.voltageCompSaturation = mk4Configuration.getNominalVoltage();

            motorConfiguration.supplyCurrLimit.currentLimit = mk4Configuration.getDriveCurrentLimit();
            motorConfiguration.supplyCurrLimit.enable = true;

            unlicensedMotor = new TalonFX(id, canbus);
            ConfigUtils.ctreConfig(() -> unlicensedMotor.configAllSettings(motorConfiguration));

            unlicensedMotor.enableVoltageCompensation(true);

            unlicensedMotor.setNeutralMode(NeutralMode.Brake);

            unlicensedMotor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            unlicensedMotor.setSensorPhase(true);
        }
    }

    @Override
    public Falcon500DriveController withPidConstants(double proportional, double integral, double derivative) {
        hasPidConstants = true;

        if (licensed) {
            motorConfiguration.Slot0.kP = proportional;
            motorConfiguration.Slot0.kI = integral;
            motorConfiguration.Slot0.kD = derivative;

            ConfigUtils.ctreProConfig(() -> licensedMotor.getConfigurator().apply(motorConfiguration.Slot0));
        }
        else {
            ConfigUtils.ctreConfig(() -> unlicensedMotor.config_kP(0, proportional));
            ConfigUtils.ctreConfig(() -> unlicensedMotor.config_kI(0, integral));
            ConfigUtils.ctreConfig(() -> unlicensedMotor.config_kD(0, derivative));
        }

        return this;
    }

    @Override
    public Falcon500DriveController withFeedforward(SimpleMotorFeedforward feedforward) {
        this.feedforward = feedforward;
        return this;
    }

    @Override
    public boolean hasFeedForward() {
        return feedforward != null;
    }

    /**
     * @param velocity desired velocity in m/s
     */
    @Override
    public void setReferenceVelocity(double velocity) {
        if (hasPidConstants) {
            double velocityRawUnits = velocity / sensorVelocityCoefficient;

            if (hasFeedForward()) {
                if (licensed) {
                    licensedMotor.setControl(new VelocityVoltage(
                            velocityRawUnits,
                            true,
                            feedforward.calculate(velocity),
                            0,
                            false
                    ));
                }
                else {
                    unlicensedMotor.set(TalonFXControlMode.Velocity, velocityRawUnits, DemandType.ArbitraryFeedForward, feedforward.calculate(velocity));
                }
            }
            else {
                if (licensed) licensedMotor.setControl(new VelocityTorqueCurrentFOC(velocityRawUnits));
                else unlicensedMotor.set(TalonFXControlMode.Velocity, velocityRawUnits);
            }
        }
        else {
            double dutyCycle = velocity / maxVelocity;

            if (licensed) licensedMotor.setControl(new DutyCycleOut(dutyCycle));
            else unlicensedMotor.set(TalonFXControlMode.PercentOutput, dutyCycle);
        }
    }

    @Override
    public double getDistanceDriven() {
        if (licensed) {
            motorPosition = motorPosition.refresh();
            return motorPosition.getValue() * sensorPositionCoefficient;
        }
        else {
            return unlicensedMotor.getSelectedSensorPosition() * sensorPositionCoefficient;
        }
    }

    @Override
    public double getStateVelocity() {
        if (licensed) {
            motorVelocity = motorVelocity.refresh();
            return motorVelocity.getValue() * sensorVelocityCoefficient;
        }
        else {
            return unlicensedMotor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }
    }

    @Override
    public void resetEncoder() {
        if (licensed) licensedMotor.setRotorPosition(0);
        else unlicensedMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void addDashboardEntries(ShuffleboardContainer container) {
        DriveController.super.addDashboardEntries(container);

        if (licensed) {
            container.addNumber("Drive Motor Temp (degrees Celsius)", () -> {
                motorTemp = motorTemp.refresh();
                return motorTemp.getValue();
            });
        }
        else {
            container.addNumber("Drive Motor Temp (degrees Celsius)", unlicensedMotor::getTemperature);
        }
    }
}