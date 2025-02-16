// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

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

  public static class CompConsts{

    public static final boolean isCompetiton = false;

  }

  public static class DrivetrainConstants{
    // Encoder Channels
    public static final int rightEncoderChanA = 0;
    public static final int rightEncoderChanB = 1;
    public static final int leftEncoderChanA = 2;
    public static final int leftEncoderChanB = 3;
    // Motor IDs
    public static final int rightFrontPort = 2;
    public static final int rightBackPort = 4;
    public static final int leftFrontPort = 1;
    public static final int leftBackPort = 3;
    // "Physics" constants (Kine/Odome)
    public static final double MeasurementPlaceholder = 0.0;

    // Misc
    public static final int multiplier = 1;
    public static final double Deadband = 0.25;

    // Path planner (Kinematics/Odometry Objects)
    
    // Kinematics

    // Values (Kine)

    public static final double trackWidth = 29;

    // Kine Obj

    // Odometry

    // Values (Odom)

    // Odom Obj

    // Create the configuration
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
    public static final double ClimbRunningValue = 0.5;
  }

  public static class ElevatorConstants{
    // Motor channels
    public static final int ElevatorMotor = 7;
    public static final int ElevatorMotorFollower = 8;
    // Elevator Motor Run Value
    public static final double ElevatorMotorRunValue = 0.4;
  }

  public static class ArmConstants{
    // Motor Channels
    public static final int ArmMotor = 9;
    public static final int ArmMotorFollower = 10;
    // Arm run value
    public static final double ArmRunValue = 0.3;
  }

  public static class IntakeConstants{
    // Motor Channels
    public static final int WristMotor = 11;
    public static final int IntakeMotor = 12;

    // Run values
    public static final double IntakeRunValue = 0.2;
    public static final double WristRunValue = 0.2;
  }
}
