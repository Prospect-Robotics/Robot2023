package com.team2813.lib.swerve.helpers;

import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;
import com.team2813.lib.swerve.controllers.drive.Falcon500DriveController;
import com.team2813.lib.swerve.controllers.drive.NeoDriveController;
import com.team2813.lib.swerve.controllers.SwerveModule;
import com.team2813.lib.swerve.controllers.steer.Falcon500SteerController;
import com.team2813.lib.swerve.controllers.steer.NeoSteerController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class Mk4SwerveModuleHelper {

    private static SteerControllerFactory<?, Falcon500SteerConfiguration<CanCoderAbsoluteConfiguration>> getFalcon500SteerFactory(Mk4ModuleConfiguration configuration) {
        return new Falcon500SteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(0.2, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }

    private static SteerControllerFactory<?, NeoSteerConfiguration<CanCoderAbsoluteConfiguration>> getNeoSteerFactory(Mk4ModuleConfiguration configuration) {
        return new NeoSteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(1.0, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);
        driveController.addDashboardEntries(container);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );
        steerController.addDashboardEntries(container);

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param canbus           Name of the CANbus; can be a SocketCAN interface (on Linux),
     *      *                  or a CANivore device name or serial number.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            String canbus,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, canbus, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);
        driveController.addDashboardEntries(container);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        canbus,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );
        steerController.addDashboardEntries(container);

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500(
                container,
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param gearRatio        The gearing configuration the module is in.
     * @param canbus           Name of the CANbus; can be a SocketCAN interface (on Linux),
     *      *                  or a CANivore device name or serial number.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            String canbus,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500(
                container,
                new Mk4ModuleConfiguration(),
                gearRatio,
                canbus,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param canbus           Name of the CANbus; can be a SocketCAN interface (on Linux),
     *      *                  or a CANivore device name or serial number.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            String canbus,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, canbus, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        canbus,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500(
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param canbus           Name of the CANbus; can be a SocketCAN interface (on Linux),
     *      *                  or a CANivore device name or serial number.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            GearRatio gearRatio,
            String canbus,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500(
                new Mk4ModuleConfiguration(),
                gearRatio,
                canbus,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        NeoDriveController driveController = new NeoDriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);
        driveController.addDashboardEntries(container);

        NeoSteerController steerController = new NeoSteerController(
                new com.team2813.lib.swerve.controllers.steer.SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );
        steerController.addDashboardEntries(container);

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createNeo(
                container,
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        NeoDriveController driveController = new NeoDriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);

        NeoSteerController steerController = new NeoSteerController(
                new com.team2813.lib.swerve.controllers.steer.SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createNeo(
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);
        driveController.addDashboardEntries(container);

        NeoSteerController steerController = new NeoSteerController(
                new com.team2813.lib.swerve.controllers.steer.SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );
        steerController.addDashboardEntries(container);

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500Neo(
                container,
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        Falcon500DriveController driveController = new Falcon500DriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);

        NeoSteerController steerController = new NeoSteerController(
                new com.team2813.lib.swerve.controllers.steer.SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createFalcon500Neo(
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        NeoDriveController driveController = new NeoDriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);
        driveController.addDashboardEntries(container);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );
        steerController.addDashboardEntries(container);

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     * Module information is displayed in the specified Shuffleboard container.
     *
     * @param container        The container to display module information in
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createNeoFalcon500(
                container,
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        NeoDriveController driveController = new NeoDriveController(driveMotorPort, gearRatio.getConfiguration(), configuration)
                .withPidConstants(drive_kP, drive_kI, drive_kD)
                .withFeedforward(driveFeedforward);

        Falcon500SteerController steerController = new Falcon500SteerController(
                new com.team2813.lib.swerve.controllers.steer.Falcon500SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),
                gearRatio.getConfiguration(),
                configuration
        );

        return new SwerveModule(driveController, steerController);
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param drive_kP         The proportional gain for the driving PID.
     * @param drive_kI         The integral gain for the driving PID.
     * @param drive_kD         The derivative gain for the driving PID.
     * @param driveFeedforward Object to calculate feedforward added to driving PID.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double drive_kP,
            double drive_kI,
            double drive_kD,
            SimpleMotorFeedforward driveFeedforward,
            double steerOffset
    ) {
        return createNeoFalcon500(
                new Mk4ModuleConfiguration(),
                gearRatio,
                driveMotorPort,
                steerMotorPort,
                steerEncoderPort,
                drive_kP,
                drive_kI,
                drive_kD,
                driveFeedforward,
                steerOffset
        );
    }
}