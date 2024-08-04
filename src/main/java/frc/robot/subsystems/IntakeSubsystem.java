// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
    private CANSparkMax motorTop;
    private CANSparkMax motorMiddle;
    private CANSparkMax motorBottom;
    private double targetBottomSpeed;
    private double targetRollerSpeed;


    public IntakeSubsystem() {
      this.motorTop = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
      this.motorMiddle = new CANSparkMax(Constants.IntakeConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom.setIdleMode(IdleMode.kBrake);
      this.motorTop.setIdleMode(IdleMode.kBrake);
      this.motorMiddle.setIdleMode(IdleMode.kBrake);
      this.targetBottomSpeed = 0.0;
      this.targetRollerSpeed = 0.0;
    }

//Add deadbands fopr the atspeed triggers because fluctuations exist

    public void setBottomSpeed(double targetBottomSpeed) {
      this.targetBottomSpeed = targetBottomSpeed * Constants.IntakeConstants.BOTTOM_MOTOR_GEAR_RATIO_COEFFICIENT;
    }

    public void setRollerSpeed(double targetRollerSpeed) {
      this.targetRollerSpeed = targetRollerSpeed * Constants.IntakeConstants.ROLLER_MOTORS_GEAR_RATIO_COEFFICIENT;
    }

    public Trigger rollerSpeedAtTarget = new Trigger (() -> (motorMiddle.get() == this.targetRollerSpeed && motorTop.get() == this.targetRollerSpeed));

    public Trigger bottomSpeedAtTarget = new Trigger(() -> (motorBottom.get() == this.targetBottomSpeed));


    @Override
    public void periodic() {
      this.motorBottom.set(this.targetBottomSpeed);
      this.motorMiddle.set(this.targetRollerSpeed);
      this.motorTop.set(this.targetRollerSpeed);
    }
}
