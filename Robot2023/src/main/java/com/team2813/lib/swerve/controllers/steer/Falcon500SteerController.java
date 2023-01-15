package com.team2813.lib.swerve.controllers.steer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.*;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class Falcon500SteerController implements SteerController {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private TalonFX unlicensedMotor;
    private com.ctre.phoenixpro.hardware.TalonFX licensedMotor;
    private final SteerEncoder absoluteEncoder;
    private final double sensorPositionCoefficient;
    private final double sensorVelocityCoefficient;
    private final TalonFXControlMode controlMode = TalonFXControlMode.Position;
    private final boolean licensed;

    private double referenceAngleRadians = 0;
    private double resetIteration = 0;
    private StatusSignalValue<Double> motorPosition;
    private StatusSignalValue<Double> motorVelocity;
    private StatusSignalValue<Double> motorTemp;

    public Falcon500SteerController(
            Falcon500SteerConfiguration steerConfiguration,
            ModuleConfiguration moduleConfiguration,
            Mk4ModuleConfiguration mk4Configuration,
            boolean isLicensed
    ) {
        licensed = isLicensed;
        CanCoderAbsoluteConfiguration absoluteEncoderConfig = steerConfiguration.getEncoderConfiguration();

        if (isLicensed) {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.MagnetOffset = absoluteEncoderConfig.getOffset() / (2 * Math.PI);
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            CANcoder cancoder = new CANcoder(absoluteEncoderConfig.getId(), steerConfiguration.getCanbus());
            ConfigUtils.ctreProConfig(() -> cancoder.getConfigurator().apply(config, 0.25));

            absoluteEncoder = new SteerEncoder(cancoder);

            sensorPositionCoefficient = 2 * Math.PI * moduleConfiguration.getSteerReduction();
            sensorVelocityCoefficient = sensorPositionCoefficient;

            com.ctre.phoenixpro.configs.TalonFXConfiguration motorConfiguration = new com.ctre.phoenixpro.configs.TalonFXConfiguration();
            motorConfiguration.Slot0.kP = 0.2;
            motorConfiguration.Slot0.kD = 0.1;

            motorConfiguration.Voltage.PeakForwardVoltage = mk4Configuration.getNominalVoltage();
            motorConfiguration.Voltage.PeakReverseVoltage = -mk4Configuration.getNominalVoltage();

            motorConfiguration.CurrentLimits.SupplyCurrentLimit = mk4Configuration.getSteerCurrentLimit();
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isSteerInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            licensedMotor = new com.ctre.phoenixpro.hardware.TalonFX(steerConfiguration.getMotorPort(), steerConfiguration.getCanbus());
            ConfigUtils.ctreProConfig(() -> licensedMotor.getConfigurator().apply(motorConfiguration, 0.25));

            ConfigUtils.ctreProConfig(() -> licensedMotor.setRotorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0.25));

            motorPosition = licensedMotor.getRotorPosition();
            motorVelocity = licensedMotor.getRotorVelocity();
            motorTemp = licensedMotor.getDeviceTemp();

            // Config status frame rates if not using CANivore (CAN FD sets optimal rates already)
            if (steerConfiguration.getCanbus().equals("")) {
                ConfigUtils.ctreProConfig(() -> motorPosition.setUpdateFrequency(4, 0.25));
                ConfigUtils.ctreProConfig(() -> motorVelocity.setUpdateFrequency(4, 0.25));
                ConfigUtils.ctreProConfig(() -> motorTemp.setUpdateFrequency(4, 0.25));
            }
        }
        else {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(absoluteEncoderConfig.getOffset());
            config.sensorDirection = false;

            CANCoder cancoder = new CANCoder(absoluteEncoderConfig.getId(), steerConfiguration.getCanbus());
            ConfigUtils.ctreConfig(() -> cancoder.configAllSettings(config, 250));

            absoluteEncoder = new SteerEncoder(cancoder);

            sensorPositionCoefficient = 2 * Math.PI / 2048 * moduleConfiguration.getSteerReduction();
            sensorVelocityCoefficient = sensorPositionCoefficient * 10;

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            motorConfiguration.slot0.kP = 0.2;
            motorConfiguration.slot0.kD = 0.1;

            motorConfiguration.voltageCompSaturation = mk4Configuration.getNominalVoltage();

            motorConfiguration.supplyCurrLimit.currentLimit = mk4Configuration.getSteerCurrentLimit();
            motorConfiguration.supplyCurrLimit.enable = true;

            unlicensedMotor = new TalonFX(steerConfiguration.getMotorPort(), steerConfiguration.getCanbus());
            ConfigUtils.ctreConfig(() -> unlicensedMotor.configAllSettings(motorConfiguration, 250));

            unlicensedMotor.enableVoltageCompensation(true);

            ConfigUtils.ctreConfig(() -> unlicensedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 250));
            unlicensedMotor.setSensorPhase(true);
            unlicensedMotor.setInverted(moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
            unlicensedMotor.setNeutralMode(NeutralMode.Brake);

            ConfigUtils.ctreConfig(() -> unlicensedMotor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0, 250));

            // Config status frame rates if not using CANivore (CAN FD sets optimal rates already)
            if (steerConfiguration.getCanbus().equals("")) {
                ConfigUtils.ctreConfig(() -> unlicensedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated,
                        250,
                        250));
                ConfigUtils.ctreConfig(() -> unlicensedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                        125,
                        250));
            }
        }
    }

    @Override
    public double getReferenceAngle() {
        return referenceAngleRadians;
    }

    @Override
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians;
        if (licensed) {
            motorPosition = motorPosition.refresh();
            currentAngleRadians = motorPosition.getValue() * sensorPositionCoefficient;
        }
        else {
            currentAngleRadians = unlicensedMotor.getSelectedSensorPosition() * sensorPositionCoefficient;
        }

        // Reset the motor's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (licensed) {
            motorVelocity = motorVelocity.refresh();
            double currentMotorVelocity = motorVelocity.getValue() * sensorVelocityCoefficient;

            if (currentMotorVelocity < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    licensedMotor.setRotorPosition(absoluteAngle / sensorPositionCoefficient);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }
        }
        else {
            if (unlicensedMotor.getSelectedSensorVelocity() * sensorVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    unlicensedMotor.setSelectedSensorPosition(absoluteAngle / sensorPositionCoefficient);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        if (licensed) {
            licensedMotor.setControl(new PositionVoltage(adjustedReferenceAngleRadians / sensorPositionCoefficient));
        }
        else {
            unlicensedMotor.set(controlMode, adjustedReferenceAngleRadians / sensorPositionCoefficient);
        }

        this.referenceAngleRadians = referenceAngleRadians;
    }

    @Override
    public double getStateAngle() {
        double motorAngleRadians;
        if (licensed) {
            motorPosition = motorPosition.refresh();
            motorAngleRadians = motorPosition.getValue() * sensorPositionCoefficient;
        }
        else {
            motorAngleRadians = unlicensedMotor.getSelectedSensorPosition() * sensorPositionCoefficient;
        }

        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    public void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Current Angle", () -> Math.toDegrees(getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(getReferenceAngle()));

        container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(absoluteEncoder.getAbsoluteAngle()));

        if (licensed) {
            container.addNumber("Steer Motor Temp (degrees Celsius)", () -> {
                motorTemp = motorTemp.refresh();
                return motorTemp.getValue();
            });
        }
        else {
            container.addNumber("Steer Motor Temp (degrees Celsius)", unlicensedMotor::getTemperature);
        }
    }
}