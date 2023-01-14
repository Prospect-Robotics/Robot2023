package com.team2813.frc2023.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.team2813.frc2023.Robot;
import com.team2813.lib.imu.Pigeon2ProWrapper;
import com.team2813.lib.swerve.controllers.SwerveModule;
import com.team2813.lib.swerve.helpers.Mk4iSwerveModuleHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Drive extends SubsystemBase {

    public static final double MAX_VELOCITY = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI; // m/s
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACKWIDTH / 2, WHEELBASE / 2); // radians per second

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

    private final String canbus = "swerve";
    private final boolean licensed = true;

    private SwerveDriveOdometry odometry;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final Pigeon2ProWrapper pigeon = new Pigeon2ProWrapper(PIGEON_ID, canbus);

    private ChassisSpeeds chassisSpeedDemand = new ChassisSpeeds(0, 0, 0);
    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public Drive() {
        double frontLeftSteerOffset = -Math.toRadians(0);
        double frontRightSteerOffset = -Math.toRadians(0);
        double backLeftSteerOffset = -Math.toRadians(0);
        double backRightSteerOffset = -Math.toRadians(0);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(0, 0),
                GearRatio.L2,
                canbus,
                FRONT_LEFT_DRIVE_ID,
                FRONT_LEFT_STEER_ID,
                FRONT_LEFT_ENCODER_ID,
                frontLeftSteerOffset,
                licensed
        );

        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(2, 0),
                GearRatio.L2,
                canbus,
                FRONT_RIGHT_DRIVE_ID,
                FRONT_RIGHT_STEER_ID,
                FRONT_RIGHT_ENCODER_ID,
                frontRightSteerOffset,
                licensed
        );

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(4, 0),
                GearRatio.L2,
                canbus,
                BACK_LEFT_DRIVE_ID,
                BACK_LEFT_STEER_ID,
                BACK_LEFT_ENCODER_ID,
                backLeftSteerOffset,
                licensed
        );

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4).withPosition(6, 0),
                GearRatio.L2,
                canbus,
                BACK_RIGHT_DRIVE_ID,
                BACK_RIGHT_STEER_ID,
                BACK_RIGHT_ENCODER_ID,
                backRightSteerOffset,
                licensed
        );

        pigeon.configMountPose(0, 0, 0);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds demand) {
        chassisSpeedDemand = demand;
    }

    public void drive(SwerveModuleState[] moduleStates) {
        this.moduleStates = moduleStates;
    }

    public void initAutonomous(PathPlannerTrajectory.PathPlannerState initialState) {
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();

        pigeon.setHeading(initialState.holonomicRotation.getDegrees());

        SwerveModulePosition[] modulePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
        odometry = new SwerveDriveOdometry(kinematics, initialState.holonomicRotation, modulePositions);
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

        if (!Robot.isAuto) moduleStates = kinematics.toSwerveModuleStates(chassisSpeedDemand);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);

        frontLeftModule.set(moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getRadians());
        frontRightModule.set(moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getRadians());
        backLeftModule.set(moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getRadians());
        backRightModule.set(moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getRadians());
    }
}
