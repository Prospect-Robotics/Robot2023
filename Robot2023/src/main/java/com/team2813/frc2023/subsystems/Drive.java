package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.team2813.lib.imu.Pigeon2Wrapper;
import com.team2813.lib.swerve.helpers.Mk4SwerveModuleHelper;
import com.team2813.lib.swerve.controllers.SwerveModule;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Drive extends SubsystemBase {

    public static final double MAX_VELOCITY = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            WHEEL_CIRCUMFERENCE; // m/s
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACKWIDTH / 2, WHEELBASE / 2); // radians per second

    private final double kP = 0.125;
    private final double kI = 0;
    private final double kD = 0;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.59869, 0.050736, 0.0021331);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2),
            // Front Right
            new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2),
            // Back Left
            new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2),
            // Back Right
            new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2)
    );

    private SwerveDriveOdometry odometry;

    private final Pigeon2Wrapper pigeon = new Pigeon2Wrapper(PIGEON_ID, "swerve");

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final DoubleLogEntry pigeonHeadingLogger;

    private ChassisSpeeds chassisSpeedDemand = new ChassisSpeeds(0, 0, 0);
    private SwerveModuleState[] states = new SwerveModuleState[4];

    private double multiplier = 1;
    private boolean loggingEnabled = false;

    public Drive() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(0, 0),
                GearRatio.L2,
                "swerve",
                FRONT_LEFT_DRIVE_ID,
                FRONT_LEFT_STEER_ID,
                FRONT_LEFT_ENCODER_ID,
                kP,
                kI,
                kD,
                feedforward,
                FRONT_LEFT_STEER_OFFSET
        );

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(2, 0),
                GearRatio.L2,
                "swerve",
                FRONT_RIGHT_DRIVE_ID,
                FRONT_RIGHT_STEER_ID,
                FRONT_RIGHT_ENCODER_ID,
                kP,
                kI,
                kD,
                feedforward,
                FRONT_RIGHT_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(4, 0),
                GearRatio.L2,
                "swerve",
                BACK_LEFT_DRIVE_ID,
                BACK_LEFT_STEER_ID,
                BACK_LEFT_ENCODER_ID,
                kP,
                kI,
                kD,
                feedforward,
                BACK_LEFT_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(6, 0),
                GearRatio.L2,
                "swerve",
                BACK_RIGHT_DRIVE_ID,
                BACK_RIGHT_STEER_ID,
                BACK_RIGHT_ENCODER_ID,
                kP,
                kI,
                kD,
                feedforward,
                BACK_RIGHT_STEER_OFFSET
        );

        pigeon.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ);

        DataLogManager.start();
        pigeonHeadingLogger = new DoubleLogEntry(DataLogManager.getLog(), "/pigeonHeading");
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        );
    }

    public void enableSlowMode(boolean enable) {
        multiplier = enable ? 0.25 : 1;
    }

    public void drive(ChassisSpeeds demand) {
        chassisSpeedDemand = demand;
    }

    public void initAutonomous(Pose2d initialPose) {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();

        pigeon.setHeading(initialPose.getRotation().getDegrees());

        SwerveModulePosition[] modulePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
        odometry = new SwerveDriveOdometry(kinematics, initialPose.getRotation(), modulePositions, initialPose);
    }

    public void resetOdometry(Pose2d newPose) {
        SwerveModulePosition[] modulePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
        odometry.resetPosition(getRotation(), modulePositions, newPose);
    }

    public void enableLogging(boolean enable) {
        loggingEnabled = enable;
    }

    @Override
    public void periodic() {
        pigeon.periodicResetCheck();
        SmartDashboard.putNumber("Current Heading (degrees)", pigeon.getHeading());

        if (odometry != null) {
            SwerveModulePosition[] modulePositions = {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
            };
            odometry.update(getRotation(), modulePositions);

            SmartDashboard.putString("Current Pose", odometry.getPoseMeters().toString());
        }

        if (loggingEnabled) pigeonHeadingLogger.append(pigeon.getPitch());

        states = kinematics.toSwerveModuleStates(chassisSpeedDemand);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

        frontLeftModule.set(states[0].speedMetersPerSecond * multiplier, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond * multiplier, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond * multiplier, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond * multiplier, states[3].angle.getRadians());
    }
}