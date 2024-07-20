// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitTimer;

public class PivotSubsystem extends SubsystemBase {

  private CANSparkMax pivotMotorMaster;
  private CANSparkMax pivotMotorSlave;

  private PIDController movePIDController;
  private double targetAngle;
  private double kP, kI, kD, kFF;

  private OrbitTimer timer;

  private TrapezoidProfile pivotMotionProfile;
  private TrapezoidProfile.State motionProfileStartState;
  private TrapezoidProfile.State motionProfileEndState;
  private double maxVelocity;

  public ArmFeedforward pivotFeedForward;

  public TrapezoidProfile.Constraints pivotMotionProfileConstraints;

  private DutyCycleEncoder absoluteEncoder;
  private double maxAcceleration;

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
    this.maxAcceleration = 140.0;

    this.pivotMotionProfileConstraints = new TrapezoidProfile.Constraints(this.maxVelocity, this.maxAcceleration);
    this.pivotMotionProfile = new TrapezoidProfile(this.pivotMotionProfileConstraints);

    if (this.pivotMotorMaster.getEncoder().setPosition(0.0) != REVLibError.kOk) {
        DriverStation.reportError("Failed to set position on ACP NEO Encoder", true);
    }

    this.kP = 0.0;
    this.kI = 0.0;
    this.kD = 0.0; 
    this.kFF = 0.0;


    this.motionProfileStartState = new TrapezoidProfile.State(this.getPivotAngle(), 0.0);
    this.motionProfileEndState = new TrapezoidProfile.State(this.getPivotAngle(), 0.0);

    this.timer = new OrbitTimer();
  }


  public double getMotorRotations() {
    return this.pivotMotorMaster.getEncoder().getPosition();
  }

  public double getPivotAngle() {
    return this.rotationsToAngleConversion(this.getMotorRotations()) + Constants.PivotConstants.HOME_POSITION;
  }

  public double rotationsToAngleConversion(double encoderPosition) {
    // encoderPosition * 360.0 = angle of motor rotation
    // angle of motor rotation * GEAR_RATIO = pivot angle
    // Pivot angle % 360 = keep range between 0-360
    return (encoderPosition * 360.0); // % 360;
}

public double getAngularVelocity() {
  return this.rotationsToAngleConversion(this.pivotMotorMaster.getEncoder().getVelocity()) / 60.0; // Units given in
                                                                                                 // RPM. divided
                                                                                                 // by 60 to get
                                                                                                 // in deg/sec
}

public boolean atTarget() {
  return Math.abs(this.targetAngle - this.getPivotAngle()) <= Constants.PivotConstants.PIVOT_DEADBAND;
}

public void setPivotAngle(double angle) {

}

public void setTargetAngle(double targetAngle) {
  if (this.targetAngle == targetAngle) {
      return;
  }

  this.targetAngle = targetAngle;

  if (this.targetAngle >= Constants.PivotConstants.MAX_ANGLE
          || this.targetAngle <= Constants.PivotConstants.MIN_ANGLE)

  {
      DriverStation.reportError("Tried to set ACP to above or below max or min; Target: " + this.targetAngle,
              true);
      System.exit(1);
  }
  else {
      this.motionProfileStartState = new TrapezoidProfile.State(this.getPivotAngle(), this.getAngularVelocity());
      this.motionProfileEndState = new TrapezoidProfile.State(this.targetAngle, 0.0);
  }

  this.movePIDController.reset();
  this.timer.start();

  System.out.println("Target angle for ACP scheduled for: " + this.targetAngle);

  }

  public double getTargetAngle() {
    return this.targetAngle;
  }

      private double calculateControlLoopOutput() {
        TrapezoidProfile.State profileTarget = this.pivotMotionProfile.calculate(this.timer.getTimeDeltaSec(),
                this.motionProfileStartState,
                this.motionProfileEndState);
        double target = profileTarget.position;
        double input = this.getPivotAngle();

        SmartDashboard.putNumber("Pivot_Profile_Position", target);
        SmartDashboard.putNumber("Pivot_Profile_Velocity", profileTarget.velocity);

        double pidOut = this.movePIDController.calculate(input, target);

        double feedforwardOutput = this.pivotFeedForward.calculate(
                Math.toRadians(profileTarget.position),
                Math.toRadians(this.getAngularVelocity()));

        SmartDashboard.putNumber("Pivot_Feedforward_Out", feedforwardOutput);

        return pidOut + feedforwardOutput;
    }

  @Override
  public void periodic() {
    
  }
}
