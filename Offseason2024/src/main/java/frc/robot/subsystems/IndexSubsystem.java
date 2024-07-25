// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  
    private CANSparkMax motorTop;
    private CANSparkMax motorBottom;
    private DigitalInput sensor;
    private double targetSpeed;


    public IndexSubsystem() {
      this.motorTop = new CANSparkMax(Constants.IndexConstants.TOP_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom = new CANSparkMax(Constants.IndexConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      this.motorBottom.setIdleMode(IdleMode.kBrake);
      this.motorTop.setIdleMode(IdleMode.kBrake);
      this.motorTop.setInverted(true);
      this.motorBottom.setInverted(true);
      this.targetSpeed = 0.0;

      this.sensor = new DigitalInput(Constants.IndexConstants.SHOOTER_SENSOR_PIN);
    }

//Add deadbands fopr the atspeed triggers because fluctuations exist

    public void setBottomSpeed(double targetSpeed) {
      this.targetSpeed = targetSpeed * Constants.IndexConstants.MOTOR_GEAR_RATIO_COEFFICIENT;
    }

    
    public void setTopSpeed(double targetSpeed) {
      this.targetSpeed = targetSpeed * Constants.IndexConstants.MOTOR_GEAR_RATIO_COEFFICIENT;
    }

    public Trigger speedAtTarget = new Trigger(() -> (Math.abs(motorBottom.get() - this.targetSpeed) <= Constants.IndexConstants.SPEED_DEADBAND && motorTop.get() == this.targetSpeed));

    public Trigger noteDetected = new Trigger (() -> (!(sensor.get())));

    @Override
    public void periodic() {
      this.motorBottom.set(this.targetSpeed);
      this.motorTop.set(this.targetSpeed);
    }
}
