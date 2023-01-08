package com.team2813.lib.swerve.controllers.steer;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;

public class SteerEncoder implements AbsoluteEncoder {
    private final CANCoder encoder;

    public SteerEncoder(CANCoder encoder) {
        this.encoder = encoder;
    }
    @Override
    public double getAbsoluteAngle() {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}