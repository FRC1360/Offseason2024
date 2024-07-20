// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkFlex motorTop;
  CANSparkFlex motorBottom;
  SparkPIDController topController;
  SparkPIDController bottomController;
  double topP, topI, topD, topFF, bottomP, bottomI, botomD, bottomFF;

  public ShooterSubsystem() {
    this.motorTop = new CANSparkFlex(0, MotorType.kBrushless);
    this.motorBottom = new CANSparkFlex(0, MotorType.kBrushless);
    this.topController = motorTop.getPIDController();
    this.bottomController = motorBottom.getPIDController();




    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
