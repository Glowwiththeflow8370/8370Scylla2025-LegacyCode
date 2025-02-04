// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Spark. */
  
  // The only subsystem not made by me (WE NEED MORE OF THSESSS)
  SparkMax elevatorMotor;
  SparkMax elevatorMotorFollower;

  SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
  SparkMaxConfig elevatorMotorFollowerConfig = new SparkMaxConfig();
  
  public Elevator() {
    elevatorMotor = new SparkMax(ElevatorConstants.ElevatorMotor, MotorType.kBrushless);
    elevatorMotorFollower = new SparkMax(ElevatorConstants.ElevatorMotorFollower, MotorType.kBrushless);  
    
    elevatorMotorConfig.idleMode(IdleMode.kBrake);

    elevatorMotorFollowerConfig.idleMode(IdleMode.kBrake);
    elevatorMotorFollowerConfig.follow(elevatorMotor);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorFollower.configure(elevatorMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void moveMotor(double x) {
    elevatorMotor.set(x);
    
  }

  // This will be a debug method for the elevator
  // It will be used for elevator height later : )
  public double getEncoderValues(){
    return 0.0;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


