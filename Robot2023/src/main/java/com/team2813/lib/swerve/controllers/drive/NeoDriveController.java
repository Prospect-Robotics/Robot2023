package com.team2813.lib.swerve.controllers.drive;

import com.revrobotics.*;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class NeoDriveController implements DriveController {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;

    private SimpleMotorFeedforward feedforward;

    public NeoDriveController(int id, ModuleConfiguration moduleConfiguration, Mk4ModuleConfiguration mk4Configuration) {
        motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setInverted(moduleConfiguration.isDriveInverted());

        ConfigUtils.revConfig(() -> motor.enableVoltageCompensation(mk4Configuration.getNominalVoltage()));

        ConfigUtils.revConfig(() -> motor.setSmartCurrentLimit((int) mk4Configuration.getDriveCurrentLimit()));

        ConfigUtils.revConfig(() -> motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100));
        ConfigUtils.revConfig(() -> motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20));
        ConfigUtils.revConfig(() -> motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20));

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        encoder = motor.getEncoder();
        double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
        encoder.setPositionConversionFactor(positionConversionFactor);
        encoder.setVelocityConversionFactor(positionConversionFactor / 60);

        pidController = motor.getPIDController();
    }

    @Override
    public NeoDriveController withPidConstants(double proportional, double integral, double derivative) {
        ConfigUtils.revConfig(() -> pidController.setP(proportional));
        ConfigUtils.revConfig(() -> pidController.setI(integral));
        ConfigUtils.revConfig(() -> pidController.setD(derivative));

        return this;
    }

    @Override
    public NeoDriveController withFeedforward(SimpleMotorFeedforward feedforward) {
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
        if (hasFeedForward()) {
            pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }
        else {
            pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        }
    }

    @Override
    public double getStateVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void addDashboardEntries(ShuffleboardContainer container) {
        DriveController.super.addDashboardEntries(container);
        container.addNumber("Drive Motor Temp (degrees Celsius)", motor::getMotorTemperature);
    }
}