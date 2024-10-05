package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

public class FetchPath {

    public String pathName;

    public FetchPath(String pathName) {
        this.pathName = pathName;
    }

    public Command getCommand() {
        return AutoBuilder.buildAuto(pathName);
    }
}