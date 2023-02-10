package com.team2813.lib.swerve.controllers.steer;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;

public class SteerEncoder implements AbsoluteEncoder {
    private CANCoder unlicensedEncoder;
    private CANcoder licensedEncoder;
    private boolean licensed;

    public SteerEncoder(CANCoder unlicensedEncoder) {
        this.unlicensedEncoder = unlicensedEncoder;
        licensed = false;
    }

    public SteerEncoder(CANcoder licensedEncoder) {
        this.licensedEncoder = licensedEncoder;
        licensed = true;
    }
    @Override
    public double getAbsoluteAngle() {
        double angle;
        if (licensed) {
            angle = 2 * Math.PI * licensedEncoder.getAbsolutePosition().getValue();
        }
        else {
            angle = Math.toRadians(unlicensedEncoder.getAbsolutePosition());
        }
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}