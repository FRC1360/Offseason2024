// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

  CANSparkMax pivotMotorLeft;
  CANSparkMax pivotMotorRight;
  ProfiledPIDController pivotPID;
  TrapezoidProfile.Constraints profileConstraints;

  final double kP;
  final double kI;
  final double kD;
  final double kV;
  final double kS;
  final double kG;
  
  final double maxAcceleration;
  final double maxVelocity;

  double targetAngle;

  public PivotSubsystem() {
    pivotMotorLeft = new CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_LEFT_ID, MotorType.kBrushless);
    pivotMotorRight = new CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_RIGHT_ID, MotorType.kBrushless);
    pivotMotorLeft.restoreFactoryDefaults();
    pivotMotorRight.restoreFactoryDefaults();
    pivotMotorRight.follow(pivotMotorLeft, true);
    pivotMotorLeft.getEncoder().setPositionConversionFactor(Constants.PivotConstants.PIVOT_ENCODER_CONVERSION_FACTOR);
    pivotMotorLeft.getEncoder().setPosition(4.4);

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kV = 0.0;
    kS = 0.0;
    kG = 0.0;

    maxAcceleration = 1.0; // degreees per second squared
    maxVelocity = 2.0; // degrees per second

    profileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    pivotPID = new ProfiledPIDController(kP, kI, kD, profileConstraints, 0.02);
    
    targetAngle = 45.0;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public double getCurrentAngle() {
    return pivotMotorLeft.getEncoder().getPosition();
  }

  public void periodic() {
    double output = pivotPID.calculate(this.getCurrentAngle(), targetAngle);
    pivotMotorLeft.setVoltage(output);
    SmartDashboard.putNumber("Target Angle", this.targetAngle);
    SmartDashboard.putNumber("Current Angle", this.getCurrentAngle());
    SmartDashboard.putNumber("Profile Angle", pivotPID.getSetpoint().position);
    SmartDashboard.putNumber("Profile Velocity", pivotPID.getSetpoint().velocity);

  }

}
