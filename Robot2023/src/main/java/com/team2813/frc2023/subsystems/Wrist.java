package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.motors.ControlMode;

import static com.team2813.frc2023.Constants.*;


public class Wrist extends Subsystem1d<Wrist.Rotations> {
    public Wrist() {
        super(new SparkMaxWrapper(MOTOR_WRIST_ID, MotorType.kBrushless, true)); //TODO: Find ot if its inverted or not

        motor.configPID(0, 0, 0); // TODO: Tune PID
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public void startZeroingWrist() {
        motor.set(ControlMode.DUTY_CYCLE, -0.98); // TODO: Test for value to put as 2nd parameter
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum Rotations implements Position {
        INTAKE(5.99),
        CUBE_OUTAKE(13.32),
        CONE_OUTAKE(13.32 ),
        RESET(0);
        // TODO: EXACT VALUES TBD (calculations might be off)

        @Override
        public double getPos() {
            return position;
        }

        final double position;

        Rotations(double position ) {
            this.position = position;
        }
    }
}