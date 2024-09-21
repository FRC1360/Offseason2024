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
  
    private CANSparkMax motorRight;
    private CANSparkMax motorLeft;
    private double targetRollerSpeed;


    public IntakeSubsystem() {
      this.motorRight = new CANSparkMax(Constants.IntakeConstants.RIGHT_ID, MotorType.kBrushless);
      this.motorLeft = new CANSparkMax(Constants.IntakeConstants.LEFT_ID, MotorType.kBrushless);

      this.motorRight.restoreFactoryDefaults();
      this.motorLeft.restoreFactoryDefaults();

      this.motorLeft.setIdleMode(IdleMode.kBrake);
      this.motorRight.setIdleMode(IdleMode.kBrake);

      this.motorLeft.setInverted(false);

      this.targetRollerSpeed = 0.0;
    }

//Add deadbands fopr the atspeed triggers because fluctuations exist

    public void setRollerSpeed(double targetRollerSpeed) {
      this.targetRollerSpeed = targetRollerSpeed;
    }

    @Override
    public void periodic() {
      this.motorLeft.set(this.targetRollerSpeed);
      this.motorRight.set(this.targetRollerSpeed);
    }
}
