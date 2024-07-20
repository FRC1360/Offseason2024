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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.OrbitTimer;

public class PivotSubsystem extends SubsystemBase {

  private CANSparkMax pivotMotorMaster;
  private CANSparkMax pivotMotorSlave;

  private PIDController movePIDController;
  private double targetAngle;
  private double kP, kI, kD, kS, kG, kV;

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
    this.pivotFeedForward = new ArmFeedforward(kS, kG, kV);

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
    this.kS = 0.0;
    this.kG = 0.0;
    this.kV = 0.0;


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

  public Trigger atTarget = new Trigger(() -> Math.abs(this.targetAngle - this.getPivotAngle()) <= Constants.PivotConstants.PIVOT_DEADBAND);
  public Trigger atHome = new Trigger(() -> Math.abs(this.getPivotAngle() - Constants.PivotConstants.HOME_POSITION) <= Constants.PivotConstants.PIVOT_DEADBAND);
  public Trigger atMaxAngle = new Trigger(() -> Math.abs(this.getPivotAngle() - Constants.PivotConstants.MAX_ANGLE) <= Constants.PivotConstants.PIVOT_DEADBAND);
  public Trigger atMinAngle = new Trigger(() -> Math.abs(this.getPivotAngle() - Constants.PivotConstants.MIN_ANGLE) <= Constants.PivotConstants.PIVOT_DEADBAND);

  public void setTargetAngle(double targetAngle) {
    if (this.targetAngle == targetAngle) {
      return;
    }

    this.targetAngle = targetAngle;
    }

  public void goToTargetAngle() {
  if (this.targetAngle >= Constants.PivotConstants.MAX_ANGLE
          || this.targetAngle <= Constants.PivotConstants.MIN_ANGLE)

  {
      DriverStation.reportError("Tried to make Pivot go above or below max or min; Target: " + this.targetAngle,
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
     updateSmartDashboard();
        // All of Control Loop motion is done within the subsystem -- simply set a
        // target angle and the subsystem will go there
        // When the motion profile is finished, the result which it outputs will be the
        // goal, making it a PID/FF control loop only
        double out = calculateControlLoopOutput();
        SmartDashboard.putNumber("ACP_Control_Loop_Out", out);
        goToTargetAngle();
        // this.setACPNormalizedVoltage(out);
        // SmartDashboard.putNumber("Current Angle: ", this.getACPAngle());
        // SmartDashboard.putNumber("Target Angle: ", true);
    }

    public void updateSmartDashboard() {

        Preferences.getDouble("pivot_Move_P_Gain", this.movePIDController.getP());
        Preferences.getDouble("pivot_Move_I_Gain", this.movePIDController.getI());
        Preferences.getDouble("pivot_Move_D_Gain", this.movePIDController.getD());
        Preferences.getDouble("pivot_Absolute_Encoder_Get", this.absoluteEncoder.get());
        Preferences.getDouble("pivot_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.getDouble("pivot_Motor_Encoder", this.pivotMotorMaster.getEncoder().getPosition());

        Preferences.getDouble("pivot kP", kP);
        Preferences.getDouble("pivot kI", kI);
        Preferences.getDouble("pivot kD", kD);
        Preferences.getDouble("pivot FeedForward kG", kG);
        Preferences.getDouble("pivot FeedForward kV", kV);
        Preferences.getDouble("pivot FeedForward kS", kS);
        SmartDashboard.putNumber("Pivot_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("pivot_Angle", this.getPivotAngle());
        SmartDashboard.putNumber("pivot_Output_Master", this.pivotMotorMaster.get());
        SmartDashboard.putNumber("pivot_Angular_Velocity", this.getAngularVelocity());
        SmartDashboard.putNumber("pivot_Absolute_Encoder_Get", this.absoluteEncoder.get());
        SmartDashboard.putNumber("pivot_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("pivot_Motor_Encoder", this.pivotMotorMaster.getEncoder().getPosition());
    }

    public void resetArmTargetAngle() {
        this.motionProfileStartState = new TrapezoidProfile.State(this.getPivotAngle(), 0.0);
        this.motionProfileEndState = new TrapezoidProfile.State(this.getPivotAngle(), 0.0);

        this.targetAngle = this.getPivotAngle();
    }
}
