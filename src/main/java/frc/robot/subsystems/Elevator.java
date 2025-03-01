// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

// import frc.robot.Constants;
// import frc.robot.Constants.ArmConstants;
// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.ElevatorConstants;

// @Logged
public class Elevator extends SubsystemBase {
  /** Creates a new Spark. */
  
  // The only subsystem not made by me (WE NEED MORE OF THSESSS)
  TalonFX elevatorMotor;
  TalonFX elevatorMotorFollower;

  TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
  TalonFXConfiguration elevatorMotorFollowerConfig = new TalonFXConfiguration();

  StatusSignal<Angle> elevatorEncoder;

  // DutyCycleEncoder ElevatorEncoder;
  
  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.ElevatorMotor);
    elevatorMotorFollower = new TalonFX(ElevatorConstants.ElevatorMotorFollower);  
    
    // Elevator Motor Config
    elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorMotorConfig.Feedback.SensorToMechanismRatio = 2.26;

    // It's followers config
    elevatorMotorFollowerConfig.CurrentLimits.StatorCurrentLimit = 80;
    
    // Apply configuration
    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorMotorFollowerConfig);

    // elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    // elevatorMotorFollower.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    // Create Encoders
    // ElevatorEncoder = new DutyCycleEncoder(ElevatorConstants.ElevatorEncoderChannel);
    
    elevatorEncoder = elevatorMotor.getPosition();
  }
  public void moveMotor(double x) {
    elevatorMotor.set(x);
    setNeutralMode(NeutralModeValue.Coast);
  }

  public void stop(){
    elevatorMotor.set(0);
    setNeutralMode(NeutralModeValue.Brake);
  }

  // This will be a debug method for the elevator
  // It will be used for elevator height later : )
  public double getEncoderValues(){
    return elevatorEncoder.getValueAsDouble() * ConversionConstants.AngleConversionValue;
  }

  public void setNeutralMode(NeutralModeValue value){
    elevatorMotor.setNeutralMode(value);
    elevatorMotorFollower.setNeutralMode(value);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoder = elevatorMotor.getPosition();

        SmartDashboard.putNumber("Elevator Position (Deg)", getEncoderValues());
    // System.out.println("Elev Enc Vals: " + getAverageEncoderValues());
  }
}


