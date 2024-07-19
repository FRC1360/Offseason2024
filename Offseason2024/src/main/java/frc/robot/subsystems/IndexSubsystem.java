// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  
    private CANSparkMax motorTop;
    private CANSparkMax motorBottom;
    private double targetSpeed;


    public IndexSubsystem() {
      this.motorTop = new CANSparkMax(Constants.IndexConstants.TOP_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom = new CANSparkMax(Constants.IndexConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      this.targetSpeed = 0.0;
    }

//Add deadbands fopr the atspeed triggers because fluctuations exist

    public void setBottomSpeed(double targetSpeed) {
      this.targetSpeed = targetSpeed * Constants.IndexConstants.MOTOR_GEAR_RATIO_COEFFICIENT;
    }

    public boolean bottomSpeedAtTraget() {
      if (motorBottom.get() == this.targetSpeed && motorTop.get() == this.targetSpeed) {
        return true;
      }
      else {
        return false;
      }
    }

    @Override
    public void periodic() {
      this.motorBottom.set(this.targetSpeed);
      this.motorTop.set(this.targetSpeed);
    }
}
