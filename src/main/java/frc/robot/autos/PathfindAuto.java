package frc.robot.autos;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PathfindAuto {

    private SwerveSubsystem swerveSubsystem;
    private PathConstraints constraints;

    private Pose2d targetPose;

    private boolean allowEnd;

    public PathfindAuto(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this(swerveSubsystem, targetPose, false);
    }

    public PathfindAuto(SwerveSubsystem swerveSubsystem, Pose2d targetPose, boolean allowEnd) {
        this.targetPose = targetPose;

        this.swerveSubsystem = swerveSubsystem;

        this.constraints = new PathConstraints(Constants.AutonConstants.maxSpeed,
                Constants.AutonConstants.maxAcceleration,
                Constants.AutonConstants.maxAngularVelocity,
                Constants.AutonConstants.maxAngularAcceleration);

        this.allowEnd = allowEnd;
    }

    public Command getCommand() {
        return new InstantCommand();
    }
}