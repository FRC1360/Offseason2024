// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

  CANSparkMax pivotMotorLeft;
  CANSparkMax pivotMotorRight;
  ProfiledPIDController pivotPID;
  TrapezoidProfile.Constraints profileConstraints;
  ArmFeedforward ffController;

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
    pivotMotorLeft.setSmartCurrentLimit(20);
    pivotMotorRight.setSmartCurrentLimit(20);

    kP = 0.15;
    kI = 0.0;
    kD = 0.0;
    kV = 2.83;
    kS = 0.0;
    kG = 0.14;

    maxAcceleration = 600.0; // degreees per second squared
    maxVelocity = 150.0; // degrees per second

    profileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    pivotPID = new ProfiledPIDController(kP, kI, kD, profileConstraints, 0.02);
    ffController = new ArmFeedforward(kS, kG, kV);
    targetAngle = 4.4;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public double getCurrentAngle() {
    return pivotMotorLeft.getEncoder().getPosition();
  }

  public void periodic() {
    double output = pivotPID.calculate(this.getCurrentAngle(), targetAngle);
    double ffOutput = ffController.calculate(Units.degreesToRadians(pivotPID.getSetpoint().position),
        Units.degreesToRadians(pivotPID.getSetpoint().velocity));
    pivotMotorLeft.setVoltage(ffOutput + output);
    SmartDashboard.putNumber("Target Angle", this.targetAngle);
    SmartDashboard.putNumber("Current Angle", this.getCurrentAngle());
    SmartDashboard.putNumber("Profile Angle", pivotPID.getSetpoint().position);
    SmartDashboard.putNumber("Profile Velocity", pivotPID.getSetpoint().velocity);
    SmartDashboard.putNumber("FF Output", ffOutput);

  }

}
