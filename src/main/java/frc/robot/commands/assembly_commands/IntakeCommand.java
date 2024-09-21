// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.assembly_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  IndexSubsystem index;

  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IndexSubsystem index) {
    addRequirements(intake, shooter, index);
    this.intake = intake;
    this.shooter = shooter;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setRollerSpeed(Constants.IntakeConstants.ROLLER_MOTORS_INTAKE_SPEED);
    index.setSpeed(Constants.IndexConstants.INDEX_INTAKE_SPEED);
    shooter.setSpeed(Constants.ShooterConstants.SHOOTER_INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerSpeed(0);
    index.setSpeed(0);
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index.noteDetected.getAsBoolean();
  }
}
