// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import java.util.function.DoubleSupplier;

public class driveCommand extends Command {

  private SwerveSubsystem swerveDrive;
  private DoubleSupplier leftX;
  private DoubleSupplier leftY;
  private DoubleSupplier rightX;


  public driveCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.driveCommand(rightX, leftY, leftX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
