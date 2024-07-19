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


    public IntakeSubsystem() {
      this.motorTop = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
      this.motorMiddle = new CANSparkMax(Constants.IntakeConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      }

    public //intake is 4:1
    //index is 4:1
    //shooter is 1:1
    //pivot 112.5:1

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
