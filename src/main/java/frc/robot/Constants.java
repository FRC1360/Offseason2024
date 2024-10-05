// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);

    public static final com.pathplanner.lib.util.PIDConstants translation = new com.pathplanner.lib.util.PIDConstants(
            1, 0, 0.0045);
    public static final com.pathplanner.lib.util.PIDConstants rotation = new com.pathplanner.lib.util.PIDConstants(
            0.05, 0.000001, 0);
    public static final double maxSpeed = 4; // m/s
    public static final double maxAcceleration = 16; // m/s^2
    public static final double maxAngularVelocity = Units.degreesToRadians(540); // d/s
    public static final double maxAngularAcceleration = Units.degreesToRadians(720); // deg/s^2
    public static final double positionTolerance = 0.025 * 20;
    public static final double angleTolerance = Math.toRadians(1) * 10;
}

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int LEFT_ID = 30;
    public static final int RIGHT_ID = 31;
    public static final double BOTTOM_MOTOR_GEAR_RATIO_COEFFICIENT = 4;
    public static final double ROLLER_MOTORS_GEAR_RATIO_COEFFICIENT = 4;
    public static final double BOTTOM_MOTOR_INTAKE_SPEED = 0.5;
    public static final double ROLLER_MOTORS_INTAKE_SPEED = 1.0;
  }

  public static class IndexConstants {
    public static final int TOP_MOTOR_ID = 50;
    public static final int BOTTOM_MOTOR_ID = 51;
    public static final double MOTOR_GEAR_RATIO_COEFFICIENT = 4;
    public static final int INDEX_SENSOR_PIN = 0;
    public static final double SPEED_DEADBAND = 0.1;
    public static final double INDEX_INTAKE_SPEED = 0.3;
    public static final double INDEX_SHOOT_SPEED = 0.6;
  }

  public static class ShooterConstants {

    public static final double VELOCITY_DEADBAND = 150;
    public static final double SHOOTER_INTAKE_SPEED = -0.15;
    public static final double SHOOTER_SHOOT_SPEED = 3250; // If it's too powerful, reduce it to 3500
    public static final double MAX_VELOCITY = 6800;
    public static final int TOP_MOTOR_ID = 60;
    public static final int BOTTOM_MOTOR_ID = 61;
  }

  public static class PivotConstants {
    public static final int PIVOT_MOTOR_LEFT_ID = 40;
    public static final int PIVOT_MOTOR_RIGHT_ID = 41;
    public static final int PIVOT_ENCODER_CHANNEL = 0;

    public static final double ABS_ENCODER_OFFSET = 0;
    public static final double PIVOT_ENCODER_CONVERSION_FACTOR = 1.0 / 145.0 * 360.0;

    public static final double HOME_POSITION = 4.4;
    public static final double PASS_POSITION = 10;
    public static final int PIVOT_DEADBAND = 2;
    public static final double MAX_ANGLE = 60;
    public static final double MIN_ANGLE = 0;
    public static final double SHOOT_ANGLE = 0;

    public static final double KG = 0;
    public static final double kS = 0;
    public static final double KV = 0;
  }

  public static class SubsystemWeights {
    public static final double ARM_WEIGHT = 18.895; // In pounds
  }

  // Gear ratios
  // intake is 4:1
  // index is 4:1
  // shooter is 1:1
  // pivot 112.5:1

}
