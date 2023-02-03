package com.team2813.frc2023;

import com.team2813.frc2023.commands.RotateCommand;
import edu.wpi.first.wpilibj2.command.Command;

import static com.team2813.frc2023.Robot.*;

public enum AutoRoutine {

    ROTATE_90("Rotate 90", new RotateCommand(90, ROBOT_CONTAINER.getDrive()));
    //BLAH("Yes", AUTO_FACTORY.buildTrajectoryAuto("someName"));

    private final String name;
    private final Command command;

    AutoRoutine(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return command;
    }
}
