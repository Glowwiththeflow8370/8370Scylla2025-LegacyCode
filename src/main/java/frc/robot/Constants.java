// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kStickControllerPort = 1;
  }

  public static class ConversionConstants{
    public static final double AngleConversionValue = 360;
  }

  public static class Channels{
    // Used for motors not used by the drivetrain

    // Climb Channels
    public static final int ClimbMotor = 5;
    public static final int ClimbMotorFollower = 6;

    // Elevator Channels
    public static final int ElevatorMotor = 7;
    public static final int ElevatorMotorFollower = 8;

    // Arm Channels
    public static final int ArmMotor = 9;
    public static final int ArmMotorFollower = 10;

    // Wrist/Intake Channels
    public static final int WristMotor = 11;
    public static final int IntakeMotor = 12;
  }

  public static class CompConsts{

    public static final boolean isCompetiton = false;

  }

  public static class DrivetrainConstants{
    // Motor IDs
    public static final int rightFrontPort = 2;
    public static final int rightBackPort = 4;
    public static final int leftFrontPort = 1;
    public static final int leftBackPort = 3;
    // "Physics" constants (Kine/Odome)
    public static final double MeasurementPlaceholder = 0.0;

    // Gear Ratios to try
    public static final double gearRatio = 0.1018;

    public static final double wheelRadius = Units.inchesToMeters(3);
    public static final double wheelCircumferenceMeters = 2 * Math.PI * (wheelRadius);
    public static final double countsPerRevolution = 2048;
    
    public static final double metersPerCount = 
    wheelCircumferenceMeters/(countsPerRevolution * gearRatio);
    // wheelCircumferenceMeters;

    // Misc
    public static final double multiplier = 0.7;
    public static final double Deadband = 0.25;

    // Stuff used in pathplanner
    
    // PID Consts
    public static final double kP = 0.01;
    public static final double kI = 0.01;
    public static final double kD = 0.01;
    // Track Width (used in something)
    public static final double trackWidth = 29;

    // Create the path planner configuration
    public static RobotConfig robotConfig;

    public DrivetrainConstants()
    {
      try{
        robotConfig = RobotConfig.fromGUISettings();
      }catch(Exception e){
        //e.printStackTrace();
        System.out.println("Error setting robot config");
      }
    }
  }

  public static class ClimbConstants{
    // Motor Channels
    public static final int ClimbMotor = 5;
    public static final int ClimbMotorFollower = 6;
    // Running Values
    public static final double ClimbRunningValue = 1;
  }

  public static class ElevatorConstants{
    // Motor channels
    public static final int ElevatorMotor = 7;
    public static final int ElevatorMotorFollower = 8;
    // Elevator Motor Run Value
    public static final double ElevatorMotorRunValue = 0.4;
    public static final double ElevatorMotorDownRunVal = -0.25;
    // Angle setpoints (stored in a list)
    public static final double[] ANGLE_SET_POINTS = {3100, 720};
  }

  public static class ArmConstants{
    // Motor Channels
    public static final int ArmMotor = 9;
    public static final int ArmMotorFollower = 10;

    // Arm Encoder Channel
    public static final int ArmEncoderChannel = 1;

    // Code Stops (Such Verbose)
    public static final double UpperBoundArmAngleCodeStop = 0.0;
    public static final double LowerBoundArmAngleCodeStop = 0.0;

    // Arm run value
    public static final double ArmRunValue = 0.3;
    // Wrist Conversion Value
    public static final double ArmAngleConversionValue = 1.826;

    public static final int ArmLimitSwitchPort = 9;
  }

  public static class IntakeConstants{
    // Motor Channels
    public static final int WristMotor = 11;
    public static final int IntakeMotor = 12;

    // Encoder Channel
    public static final int WristEncoderChannel = 2;

    // Run values
    public static final double IntakeRunValue = 1;
    public static final double WristRunValue = 0.4;
    public static final int IntakeMotorFollower = 13;
  }
}
