package com.team2813.frc2023.subsystems;


import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.SparkMaxWrapper;
import static com.team2813.frc2023.Constants.*;

public class Pivot extends Subsystem1d<Pivot.Rotations> {

    public Pivot() {
        super(new SparkMaxWrapper(MOTOR_WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless, true)); //TODO: Find ot if its inverted or not

        motor.configPID(0, 0, 0);
    }
    public enum Rotations implements Position {
        STARTING_CONFIGURATION(0),
        HIGH(50),
        MID(57),
        INTAKE(63),

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
