package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.motors.TalonFXProWrapper;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;
import static com.team2813.frc2023.Constants.OperatorConstants.*;

public class Intake extends SubsystemBase {
    public final SparkMaxWrapper motor = new SparkMaxWrapper(INTAKE_ID, true);

    public Intake () {

    }

    public void intake () {}

    public void outtake() {}

}
