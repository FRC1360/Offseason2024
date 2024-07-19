// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
    private CANSparkMax motorTop;
    private CANSparkMax motorMiddle;
    private CANSparkMax motorBottom;
    private double targetSpeedBottom;


    public IntakeSubsystem() {
      this.motorTop = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
      this.motorMiddle = new CANSparkMax(Constants.IntakeConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      this.targetSpeedBottom = 0.0;
    }

    public void setBottomSpeed(targetSpeedBottom) {
      this.motorBottom.set(targetSpeedBottom);
    }

    public boolean bottomSpeedAtTraget(targetSpeedBottom) {
      if (motorBottom.get() == targetSpeedBottom) {
        return true;
      }
      else {
        return false;
      }
    }

    @Override
    public void periodic() {
      setBottomSpeed(0.0);
    }
}
