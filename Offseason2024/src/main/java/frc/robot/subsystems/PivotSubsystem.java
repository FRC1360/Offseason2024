// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

  private CANSparkMax pivotMotorMaster;
  private CANSparkMax pivotMotorSlave;

  private PIDController movePIDController;
  private double kP, kI, kD, kFF;

  private TrapezoidProfile pivotMotionProfile;
  private TrapezoidProfile.State motionProfileStartState;
  private TrapezoidProfile.State motionProfileEndState;
  private double maxVelocity;

  public TrapezoidProfile.Constraints pivotMotionProfileConstraints;

  private DutyCycleEncoder absoluteEncoder;

  public PivotSubsystem() {
    this.pivotMotorMaster = new CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_MASTER_ID, MotorType.kBrushless);
    this.pivotMotorSlave = new CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_SLAVE_ID, MotorType.kBrushless);
    this.pivotMotorMaster.setIdleMode(IdleMode.kBrake);
    this.pivotMotorSlave.setIdleMode(IdleMode.kBrake);

    this.pivotMotorMaster.getEncoder().setPositionConversionFactor(Constants.PivotConstants.PIVOT_ENCODER_CONVERSION_FACTOR);
    this.pivotMotorSlave.getEncoder().setVelocityConversionFactor(Constants.PivotConstants.PIVOT_ENCODER_CONVERSION_FACTOR);

    this.pivotMotorSlave.follow(this.pivotMotorMaster);

    this.absoluteEncoder = new DutyCycleEncoder(Constants.PivotConstants.PIVOT_ENCODER_CHANNEL);

    this.movePIDController = new PIDController(kP, kI, kD);

    this.maxVelocity = 85.0; //the units are deg/sec for velocity and deg/ sec^2 for acceleration

    this.pivotMotionProfileConstraints = new TrapezoidProfile.Constraints(this.maxVelocity, 140.0);
    this.pivotMotionProfile = new TrapezoidProfile(this.pivotMotionProfileConstraints);

    if (this.pivotMotorMaster.getEncoder().setPosition(0.0) != REVLibError.kOk) {
        DriverStation.reportError("Failed to set position on ACP NEO Encoder", true);
    }

    this.kP = 0.0;
    this.kI = 0.0;
    this.kD = 0.0; 
    this.kFF = 0.0;


    this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), 0.0); // this.getACPAngle(), 0.0);
    this.motionProfileEndState = new TrapezoidProfile.State(this.getACPAngle(), 0.0);
  }


  public double getMotorRotations() {
    return this.pivotMotorMaster.getEncoder().getPosition();
  }

  public double getACPAngle() {
    return this.rotationsToAngleConversion(this.getMotorRotations()) + Constants.PivotConstants.HOME_POSITION_ACP;
  }

  public double rotationsToAngleConversion(double encoderPosition) {
    // encoderPosition * 360.0 = angle of motor rotation
    // angle of motor rotation * GEAR_RATIO = ACP angle
    // ACP angle % 360 = keep range between 0-360
    return (encoderPosition * 360.0); // % 360;
}

  @Override
  public void periodic() {

  }
}
