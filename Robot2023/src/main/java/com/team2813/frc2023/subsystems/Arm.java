package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.SparkMaxWrapper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2813.frc2023.subsystems.Subsystem1d;
import com.team2813.lib.solenoid.SolenoidGroup;

public class Arm extends Subsystem1d {

    private final SolenoidGroup pistons = new SolenoidGroup(PCM_ID, PneumaticsModuleType.CTREPCM, 0, 1);

    public Arm() {
        super(new SparkMaxWrapper(CLIMBER_ID, SparkMaxInvertType.Clockwise));

        motor.configPID(0.4, 0, 0); //probably will need to change
        motor.configMotionMagic(30000, 30000); // max vel in ticks/100ms

        setPosition(Position.RETRACTED);
    }
   
    public void periodic() {
        //add as needed
    }

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05; //probably change value
    }

    public void extendPistons() {
        pistons.set(SolenoidGroup.PistonState.EXTENDED);
    }

    public void retractPistons() {
        pistons.set(SolenoidGroup.PistonState.RETRACTED);
    }

    public void startLoweringClimber() {
        motor.set(ControlMode.DUTY_CYCLE, -0.98); //probably change value
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum extensionLength extends Subsystem1d.Position {
        INTAKE(26),
        MIDDLE(62.25),
        TOP(136.6),
        DOUBLE_SUBSTATION(80);
    }
    // void extend ()


}
