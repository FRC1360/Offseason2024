// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.assembly_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.OrbitTimer;

public class PassCommand extends Command {
  
  IndexSubsystem index;
  ShooterSubsystem shooter;
  PivotSubsystem pivot;

  public PassCommand(IndexSubsystem index, ShooterSubsystem shooter, PivotSubsystem pivot) {
    addRequirements(index, shooter, pivot);
    this.index = index;
    this.shooter = shooter;
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.setSpeed(Constants.IndexConstants.INDEX_SHOOT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setSpeed(0);
    shooter.stopShooter();
    pivot.setTargetAngle(Constants.PivotConstants.PASS_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
