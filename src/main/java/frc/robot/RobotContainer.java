// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.FetchPath;
import frc.robot.commands.assembly_commands.FireCommand;
import frc.robot.commands.assembly_commands.PassCommand;
import frc.robot.commands.assembly_commands.PrepFireAutoCommand;
import frc.robot.commands.assembly_commands.IntakeCommand;
import frc.robot.commands.assembly_commands.PrepFireCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverXbox = new CommandXboxController(2);
    private final CommandJoystick leftJoystick = new CommandJoystick(0);
    private final CommandJoystick rightJoystick = new CommandJoystick(1);
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    final IntakeSubsystem intake = new IntakeSubsystem();
    final IndexSubsystem index = new IndexSubsystem();
    final ShooterSubsystem shooter = new ShooterSubsystem();
    final PivotSubsystem pivot = new PivotSubsystem();
    public SendableChooser<Command> autoChooser;
    public ArrayList<Command> tempInitAutos = new ArrayList<>();
    final double timeoutTime = 0.5;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());

    // leftJoystick.button(2).onTrue(new InstantCommand(() ->
    // pivot.setTargetAngle(4.4)));
    // leftJoystick.button(3).onTrue(new InstantCommand(() ->
    // pivot.setTargetAngle(45)));
    // leftJoystick.button(1).onTrue((new InstantCommand(() ->
    // pivot.setTargetAngle(Constants.PivotConstants.HOME_POSITION))));
    // leftJoystick.button(1)
    // .and(() -> !((index.noteDetected).getAsBoolean()))
    // .whileTrue(
    // (new InstantCommand(() ->
    // index.setBottomSpeed(Constants.IndexConstants.BOTTOM_MOTOR_INTAKE_SPEED)))
    // .andThen(new InstantCommand(() ->
    // index.setTopSpeed(Constants.IndexConstants.TOP_MOTOR_INTAKE_SPEED))))
    // .onFalse(
    // (new InstantCommand(() -> index.setBottomSpeed(0.0)))
    // .andThen(new InstantCommand(() -> index.setTopSpeed(0.0))));

    /* drivebase.aimAtSpeaker(0.1) */

    // rightJoystick.button(7).onTrue(new InstantCommand(() ->
    // drivebase.turnToSpeaker()));

    // leftJoystick.button(2).and(index.noteDetected).onTrue(
    // (new PrepFireCommand(shooter, pivot, drivebase))
    // .andThen(new PassCommand(index, shooter, pivot).withTimeout(1))
    // ).onFalse(
    // new InstantCommand(() -> shooter.stopShooter())
    // .andThen( new InstantCommand(() ->
    // pivot.setTargetAngle(Constants.PivotConstants.HOME_POSITION))
    // ));

    public RobotContainer() {
        // Configure the trigger bindings
        loadAllAutos();
        initalizeAutoChooser();
        configureBindings();

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the rotational velocity
        // buttons are quick rotation positions to different ways to face
        // WARNING: default buttons are on the same buttons as the ones defined in
        // configureBindings
        /*
         * AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
         * () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
         * OperatorConstants.LEFT_Y_DEADBAND),
         * () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
         * OperatorConstants.LEFT_X_DEADBAND),
         * () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
         * OperatorConstants.RIGHT_X_DEADBAND),
         * driverXbox.getHID()::getYButtonPressed,
         * driverXbox.getHID()::getAButtonPressed,
         * driverXbox.getHID()::getXButtonPressed,
         * driverXbox.getHID()::getBButtonPressed);
         */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation

        /*
         * Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
         * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
         * OperatorConstants.LEFT_Y_DEADBAND),
         * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
         * OperatorConstants.LEFT_X_DEADBAND),
         * () -> driverXbox.getRightX(),
         * () -> driverXbox.getRightY());
         */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot


        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand( // Xbox controller has to be inverted
                                                                             // because it in itself is inverted. It's
                                                                             // weird :(
                () -> deadband(leftJoystick.getY() * -1,
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> deadband(leftJoystick.getX() * -1,
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> deadband(rightJoystick.getX(), 0.1) * -1);

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> deadband(driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> deadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

        this.tempInitAutos = new ArrayList<Command>();

        rightJoystick.button(1).and(index.noteDetected).and(drivebase.seeSpeaker).onTrue(
                (new PrepFireCommand(shooter, pivot, drivebase))
                        .andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));

        rightJoystick.button(1).and(index.noteDetected).onFalse(
                        new InstantCommand(() -> shooter.stopShooter())
                                .andThen(new InstantCommand(
                                        () -> pivot.setTargetAngle(Constants.PivotConstants.HOME_POSITION))));

        leftJoystick.button(1).and(() -> !((index.noteDetected).getAsBoolean())).onTrue(
                (new InstantCommand(() -> intake.setRollerSpeed(Constants.IntakeConstants.ROLLER_MOTORS_INTAKE_SPEED)))
                        .andThen(new InstantCommand(() -> index.setSpeed(Constants.IndexConstants.INDEX_INTAKE_SPEED))))
                .onFalse(
                        (new InstantCommand(() -> index.setSpeed(0.0))).andThen(
                                new InstantCommand(() -> intake.setRollerSpeed(0.0))));

        rightJoystick.button(7).onTrue(
                (new InstantCommand(() -> intake.setRollerSpeed(Constants.IntakeConstants.ROLLER_MOTORS_INTAKE_SPEED * -1)))
                        .andThen(new InstantCommand(() -> index.setSpeed(Constants.IndexConstants.INDEX_INTAKE_SPEED * -1))))
                .onFalse(
                        (new InstantCommand(() -> index.setSpeed(0.0))).andThen(
                                new InstantCommand(() -> intake.setRollerSpeed(0.0))));

        leftJoystick.button(2).and(index.noteDetected).onTrue(new PrepFireAutoCommand(10, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        rightJoystick.button(3).and(index.noteDetected).onTrue(new PrepFireAutoCommand(55, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        rightJoystick.button(2).whileTrue(drivebase.aimAtSpeaker(0.01));

        rightJoystick.button(4).whileTrue(drivebase.aimAtAmp(0.01));

        leftJoystick.button(7).onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

        /*
         * leftJoystick.button(10).onTrue((Commands.runOnce(drivebase::zeroGyro)));
         * rightJoystick.button(11).onTrue(Commands.runOnce(drivebase::
         * addFakeVisionReading));
         * rightJoystick.button(10).whileTrue(
         * Commands.deferredProxy(() -> drivebase.driveToPose(
         * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
         * ));
         * driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
         */
        // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
        // drivebase).repeatedly());

        // leftJoystick.button(2).onTrue(new InstantCommand(() ->
        // pivot.setTargetAngle(4.4)));
        // leftJoystick.button(3).onTrue(new InstantCommand(() ->
        // pivot.setTargetAngle(45)));
        // leftJoystick.button(1).onTrue((new InstantCommand(() ->
        // pivot.setTargetAngle(Constants.PivotConstants.HOME_POSITION))));
        // leftJoystick.button(1)
        // .and(() -> !((index.noteDetected).getAsBoolean()))
        // .whileTrue(
        // (new InstantCommand(() ->
        // index.setBottomSpeed(Constants.IndexConstants.BOTTOM_MOTOR_INTAKE_SPEED)))
        // .andThen(new InstantCommand(() ->
        // index.setTopSpeed(Constants.IndexConstants.TOP_MOTOR_INTAKE_SPEED))))
        // .onFalse(
        // (new InstantCommand(() -> index.setBottomSpeed(0.0)))
        // .andThen(new InstantCommand(() -> index.setTopSpeed(0.0))));
    }

    /*
     * .onFalse((new InstantCommand(() -> index.setBottomSpeed(0.0)))
     * .andThen(new InstantCommand(() -> index.setTopSpeed(0.0)))
     * .andThen(new InstantCommand(() -> shooter.setVelocity(0.0))))
     */
    public void loadAllAutos() {
        this.tempInitAutos.clear(); // in case if robot is not power cycled, data within class are typically cached

        NamedCommands.registerCommand("Fire55", new PrepFireAutoCommand(50, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        NamedCommands.registerCommand("Fire48", new PrepFireAutoCommand(48, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        NamedCommands.registerCommand("Fire30", new PrepFireAutoCommand(35, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        NamedCommands.registerCommand("Fire36", new PrepFireAutoCommand(35.5, shooter, pivot).andThen(new FireCommand(index, shooter, pivot).withTimeout(timeoutTime)));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake, shooter, index));

        System.out.println(AutoBuilder.getAllAutoNames());
        for (String pathName : AutoBuilder.getAllAutoNames()) {
            this.tempInitAutos.add(new FetchPath(pathName).getCommand());
        }
    }

    public void initalizeAutoChooser() {
        this.autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(this.autoChooser);
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return this.autoChooser.getSelected();
        return this.autoChooser.getSelected();
        // return new RepeatCommand(new DefaultDriveCommand(swerveSubsystem, () ->
        // -0.25, () -> 0.0, () -> 0.0, right_controller));
    }

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        /*
         * leftJoystick.button(10).onTrue((Commands.runOnce(drivebase::zeroGyro)));
         * rightJoystick.button(11).onTrue(Commands.runOnce(drivebase::
         * addFakeVisionReading));
         * rightJoystick.button(10).whileTrue(
         * Commands.deferredProxy(() -> drivebase.driveToPose(
         * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
         * ));
         * driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
         */
        driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
                drivebase).repeatedly());

        // leftJoystick.button(1).onTrue((new InstantCommand(() ->
        // pivot.setTargetAngle(Constants.PivotConstants.HOME_POSITION))));
        // leftJoystick.button(1)
        // .and(() -> !((index.noteDetected).getAsBoolean()))
        // .whileTrue(
        // (new InstantCommand(() ->
        // index.setBottomSpeed(Constants.IndexConstants.BOTTOM_MOTOR_INTAKE_SPEED)))
        // .andThen(new InstantCommand(() ->
        // index.setTopSpeed(Constants.IndexConstants.TOP_MOTOR_INTAKE_SPEED))))
        // .onFalse(
        // (new InstantCommand(() -> index.setBottomSpeed(0.0)))
        // .andThen(new InstantCommand(() -> index.setTopSpeed(0.0))));

        // rightJoystick.button(1)./*and(index.noteDetected)*/onTrue(
        // (new PrepFireCommand(shooter))
        // .andThen(new FireCommand(index, shooter).withTimeout(5))
        // ).onFalse(
        // new InstantCommand(() -> shooter.stopShooter())
        // );

        // leftJoystick.button(1).and(() ->
        // !((index.noteDetected).getAsBoolean())).onTrue(
        // (new InstantCommand(() ->
        // index.setSpeed(Constants.IndexConstants.INDEX_INTAKE_SPEED)))
        // )
        // .onFalse(
        // (new InstantCommand(() -> index.setSpeed(0.0)))
        // );
        /*
         * .onFalse((new InstantCommand(() -> index.setBottomSpeed(0.0)))
         * .andThen(new InstantCommand(() -> index.setTopSpeed(0.0)))
         * .andThen(new InstantCommand(() -> shooter.setVelocity(0.0))))
         */;

    }

    private static double deadband(double input, double deadband) {
        double slope = 1 / (1-deadband); // m = rise/run
        double offset = 1 - slope; // b = y - mx
        if (input < 0.0) {
            return Math.abs(input) > deadband? (-1 * (slope * Math.abs(input) + offset)) : 0.0;
        } else if (input > 0.0) {
            return Math.abs(input) > deadband? (slope * Math.abs(input) + offset): 0.0;
        } else {
            return 0.0;
        }
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     *//*
        * public Command getAutonomousCommand()
        * {
        * // An example command will be run in autonomous
        * //return drivebase.getAutonomousCommand("New Auto");
        * return null;
        * }
        * 
        * public void setDriveMode()
        * {
        * //drivebase.setDefaultCommand();
        * }
        * 
        * public void setMotorBrake(boolean brake)
        * {
        * //drivebase.setMotorBrake(brake);
        * }
        */
}
