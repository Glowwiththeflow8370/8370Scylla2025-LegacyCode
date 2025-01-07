// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  TalonFX rightFront, rightBack, leftFront, leftBack;
  Encoder rightEncoder, leftEncoder;

  // For Use later (The current method for inverting
  // Is depreciated)
  TalonFXConfiguration leftConfig;
  // Ill most likely code this to be more 
  // specific in terms of function
  TalonFXConfiguration rightConfig;
  
  public Drivetrain() {
    // Encoders (test them like this first, 
    // then figure out how to use w spark maxes)

    // Right encoder
    rightEncoder = new Encoder(DrivetrainConstants.rightEncoderChanA, 
    DrivetrainConstants.rightEncoderChanB);
    // Left encoder
    leftEncoder = new Encoder(DrivetrainConstants.leftEncoderChanA,
    DrivetrainConstants.leftEncoderChanB);
    
    // Motors
    rightFront = new TalonFX(DrivetrainConstants.rightFrontPort);
    leftFront = new TalonFX(DrivetrainConstants.leftFrontPort);

    rightBack = new TalonFX(DrivetrainConstants.rightBackPort);
    leftBack = new TalonFX(DrivetrainConstants.leftBackPort);

    // Create and set configurations (For left motor, I may have to specify the right motor)
    leftConfig = new TalonFXConfiguration();
    // Invert left motor
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Create and set configurations (For right motor)
    rightConfig = new TalonFXConfiguration();
    // Set the right motor to turn in the opposite direction of the left motor
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply the configuration thing needed
    rightFront.getConfigurator().apply(rightConfig);
    leftFront.getConfigurator().apply(leftConfig);

    // Get the back motors to follow their respective front motors
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));
    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));

  }

  public void tank(double x, double y){
    // Multiplier in constants in case it is needed
    rightFront.set(x);
    leftFront.set(y);
  }

  public double getAverageEncoderValues(){
    return ((rightEncoder.getDistance() + leftEncoder.getDistance())/2.0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
