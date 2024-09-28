// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.assembly_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepFireCommand extends Command {
  
  ShooterSubsystem shooter;
  PivotSubsystem pivot;

  public PrepFireCommand(ShooterSubsystem shooter, PivotSubsystem pivot) {
    addRequirements(shooter, pivot);
    this.shooter = shooter;
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setVelocity(Constants.ShooterConstants.SHOOTER_SHOOT_SPEED);
    pivot.setTargetAngle(55.0);  //55.0 for at speaker shot
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.atTargetVelocity.getAsBoolean() && pivot.atTargetAngle.getAsBoolean();
  }
}
