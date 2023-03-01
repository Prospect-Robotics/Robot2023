package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.util.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
    private final Drive drive;
    private final Limelight limelight = Limelight.getInstance();

    private boolean startedClimb = false;
    private boolean finishedClimb = false;
    private double movementSpeed = 1.5;
    private double stationX;
    public AutoBalanceCommand(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        limelight.setLights(true);

        switch (DriverStation.getAlliance()) {
            case Red:
                stationX = 12.7046;
                break;
            case Blue:
                stationX = 3.8354;
                break;
            default:
                break;
        }
        limelight.getPosition().ifPresentOrElse((Pose2d pose) -> {
            if (!(pose.getX() < stationX)) {
                movementSpeed = -movementSpeed;
            }
        }, () -> {
            limelight.setLights(false);
            if (!(drive.getPose().getX() < stationX)) {
                movementSpeed = -movementSpeed;
            }
        });
    }
    @Override
    public void execute() {
        double pitch = drive.getPigeon().getPitch();
        if (-1 > pitch || pitch > 1) {
            startedClimb = true;
        }
        if (startedClimb) {
            if (pitch < -2.5) {
                ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                    movementSpeed * 0.75,
                    0.0,
                    0.0,
                    drive.getRotation()
                );
                drive.drive(speed);
            } else if (pitch > 2.5) {
                ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                    -movementSpeed * 0.75,
                    0.0,
                    0.0,
                    drive.getRotation()
                );
                drive.drive(speed);
            } else {
                if (pitch < 0) {
                    ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                        movementSpeed * 0.25,
                        0.0,
                        0.0,
                        drive.getRotation()
                    );
                    drive.drive(speed);
                } else if (pitch > 0) {
                    ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                        -movementSpeed * 0.25,
                        0.0,
                        0.0,
                        drive.getRotation()
                    );
                    drive.drive(speed);
                }
                finishedClimb = true;
            }
        } else {
            ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
                movementSpeed,
                0.0,
                0.0,
                drive.getRotation()
            );
            drive.drive(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return finishedClimb;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                                                                     0.0,
                                                                     0.0,
                                                                     drive.getRotation());
        drive.drive(speed);
        limelight.setLights(false);
    }
}
