package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Intake extends SubsystemBase {

    private final TalonFXWrapper intakeMotor = new TalonFXWrapper(INTAKE_MOTOR_ID, TalonFXInvertType.CounterClockwise);

    public Intake() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void intakeCube() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, 0.7);
    }

    public void placeCube() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, -0.2);
    }

    public void shootCube() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, -1);
    }

    public void intakeCone() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, -0.7);
    }

    public void placeCone() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, 0.4);
    }

    public void stop() {
        intakeMotor.set(ControlMode.DUTY_CYCLE, 0);
    }
}
