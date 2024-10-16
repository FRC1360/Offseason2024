// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkFlex motorTop;
  CANSparkFlex motorBottom;
  SparkPIDController topController;
  SparkPIDController bottomController;
  double targetVelocity;
  double currentTopVelocity;
  double currentBottomVelocity;
  double topP, topI, topD, topFF, bottomP, bottomI, botomD, bottomFF;

  public ShooterSubsystem() {
    this.motorTop = new CANSparkFlex(Constants.ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    this.motorBottom = new CANSparkFlex(Constants.ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

    this.motorTop.restoreFactoryDefaults();
    this.motorBottom.restoreFactoryDefaults();

    this.topController = motorTop.getPIDController();
    this.bottomController = motorBottom.getPIDController();
    this.motorBottom.setClosedLoopRampRate(0);
    this.motorBottom.setOpenLoopRampRate(0);
    this.motorTop.setClosedLoopRampRate(0);
    this.motorTop.setOpenLoopRampRate(0);
    this.motorBottom.setIdleMode(IdleMode.kCoast);
    this.motorTop.setIdleMode(IdleMode.kCoast);
    this.motorBottom.setSmartCurrentLimit(80);
    this.motorTop.setSmartCurrentLimit(80);
    this.motorTop.setInverted(true);
    this.motorBottom.setInverted(true);

    this.targetVelocity = 0.0;
    this.currentTopVelocity = 0.0;
    this.currentBottomVelocity = 0.0;

    this.topP = 0.00085; // 0.0008
    this.topI = 0.0;
    this.topD = 0.0;
    this.topFF = 0.000145; // 0.000145
    this.bottomP = 0.0005; // 000012
    this.bottomI = 0.0;
    this.botomD = 0.0;
    this.bottomFF = 0.00015; // 00015

    this.topController.setP(this.topP);
    this.topController.setI(this.topI);
    this.topController.setD(this.topD);
    this.topController.setFF(this.topFF);
    this.bottomController.setP(this.bottomP);
    this.bottomController.setI(this.bottomI);
    this.bottomController.setD(this.botomD);
    this.bottomController.setFF(this.bottomFF);
  }

  Trigger atMaxVelocity = new Trigger(() -> this.currentTopVelocity >= Constants.ShooterConstants.MAX_VELOCITY
      && this.currentBottomVelocity >= Constants.ShooterConstants.MAX_VELOCITY);

  public void setVelocity(double targetVelocity) {
    if (targetVelocity >= Constants.ShooterConstants.MAX_VELOCITY) {
      this.targetVelocity = Constants.ShooterConstants.MAX_VELOCITY - 100;
      DriverStation.reportError("tried to set shooter wheels to value greater than max velocity", true);
      return;
    }
    this.targetVelocity = targetVelocity;
  }

  public void stopShooter() {
    this.targetVelocity = 1000.0;
  }

  public void setSpeed(double speed) {
    motorBottom.set(speed);
    motorTop.set(speed);
  }

  public double getTopVelocity() {
    this.currentTopVelocity = this.motorTop.getEncoder().getVelocity();
    return this.currentTopVelocity;
  }

  public double getBottomVelocity() {
    this.currentBottomVelocity = this.motorBottom.getEncoder().getVelocity();
    return this.currentBottomVelocity;
  }

  public Trigger atTargetVelocity = new Trigger(() -> (Math
      .abs(this.currentTopVelocity - this.targetVelocity) <= Constants.ShooterConstants.VELOCITY_DEADBAND)
      && (Math.abs(this.currentBottomVelocity - this.targetVelocity) <= Constants.ShooterConstants.VELOCITY_DEADBAND));

  @Override
  public void periodic() {
    this.topController.setReference(this.targetVelocity, ControlType.kVelocity);
    this.bottomController.setReference(this.targetVelocity, ControlType.kVelocity);

    // this.motorTop.set(this.targetVelocity);
    // this.motorBottom.set(this.targetVelocity);

    if (this.atMaxVelocity.getAsBoolean())
      stopShooter();
    SmartDashboard.putNumber("Shooter top current velocity", getTopVelocity());
    SmartDashboard.putNumber("Shooter bottom current velocity", getBottomVelocity());
    SmartDashboard.putNumber("Target shooter velocity", this.targetVelocity);
  }
}
