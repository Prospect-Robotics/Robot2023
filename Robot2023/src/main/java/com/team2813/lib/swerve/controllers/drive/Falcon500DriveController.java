package com.team2813.lib.swerve.controllers.drive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class Falcon500DriveController implements DriveController {
    private final TalonFX motor;
    private final double sensorPositionCoefficient;
    private final double sensorVelocityCoefficient;

    private SimpleMotorFeedforward feedforward;

    public Falcon500DriveController(int id, ModuleConfiguration moduleConfiguration, Mk4ModuleConfiguration mk4Configuration) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / 2048;
        sensorVelocityCoefficient = sensorPositionCoefficient * 10;

        motorConfiguration.voltageCompSaturation = mk4Configuration.getNominalVoltage();

        motorConfiguration.supplyCurrLimit.currentLimit = mk4Configuration.getDriveCurrentLimit();
        motorConfiguration.supplyCurrLimit.enable = true;

        motor = new TalonFX(id);
        ConfigUtils.ctreConfig(() -> motor.configAllSettings(motorConfiguration));

        motor.enableVoltageCompensation(true);

        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        motor.setSensorPhase(true);

        ConfigUtils.ctreConfig(
                () -> motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 250, 250)
        );
    }

    public Falcon500DriveController(int id, String canbus, ModuleConfiguration moduleConfiguration, Mk4ModuleConfiguration mk4Configuration) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / 2048;
        sensorVelocityCoefficient = sensorPositionCoefficient * 10;

        motorConfiguration.voltageCompSaturation = mk4Configuration.getNominalVoltage();

        motorConfiguration.supplyCurrLimit.currentLimit = mk4Configuration.getDriveCurrentLimit();
        motorConfiguration.supplyCurrLimit.enable = true;

        motor = new TalonFX(id, canbus);
        ConfigUtils.ctreConfig(() -> motor.configAllSettings(motorConfiguration));

        motor.enableVoltageCompensation(true);

        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        motor.setSensorPhase(true);

        ConfigUtils.ctreConfig(
                () -> motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 250, 250)
        );
        ConfigUtils.ctreConfig(
                () -> motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 125, 250)
        );
    }

    @Override
    public Falcon500DriveController withPidConstants(double proportional, double integral, double derivative) {
        ConfigUtils.ctreConfig(() -> motor.config_kP(0, proportional));
        ConfigUtils.ctreConfig(() -> motor.config_kI(0, integral));
        ConfigUtils.ctreConfig(() -> motor.config_kD(0, derivative));

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

    // velocity in m/s
    @Override
    public void setReferenceVelocity(double velocity) {
        double velocityRawUnits = velocity / sensorVelocityCoefficient; // convert from m/s to ticks/100ms

        if (hasFeedForward()) {
            motor.set(TalonFXControlMode.Velocity, velocityRawUnits, DemandType.ArbitraryFeedForward, feedforward.calculate(velocity));
        }
        else {
            motor.set(TalonFXControlMode.Velocity, velocityRawUnits);
        }
    }

    @Override
    public double getDistanceDriven() {
        return motor.getSelectedSensorPosition() * sensorPositionCoefficient;
    }

    @Override
    public double getStateVelocity() {
        return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
    }

    @Override
    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void addDashboardEntries(ShuffleboardContainer container) {
        DriveController.super.addDashboardEntries(container);
        container.addNumber("Drive Motor Temp (degrees Celsius)", motor::getTemperature);
    }
}