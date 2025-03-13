// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.IntakeConstants;

// @Logged
public class Wrist extends SubsystemBase{
  private SparkMax WristMotor;
  private SparkMaxConfig WristMotorConfig;

  private DutyCycleEncoder WristEncoder;
  
  /** Creates a new Wrist. */
  public Wrist() {
    // Initialize Wrist Motor
    WristMotor = new SparkMax(IntakeConstants.WristMotor, MotorType.kBrushless);
    // Initialize Wrist Encoder
    WristEncoder = new DutyCycleEncoder(IntakeConstants.WristEncoderChannel);

    // Create Configurations
    WristMotorConfig = new SparkMaxConfig();
    WristMotorConfig.idleMode(IdleMode.kBrake);
    WristMotorConfig.absoluteEncoder
      .positionConversionFactor(ConversionConstants.AngleConversionValue);
    
    // Apply Configs to wrist motor
    WristMotor.configure(WristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Rotates the wrist
  public void rotateWrist(double value){
    WristMotor.set(value);
  }

  // Stops the wrist
  public void stopWrist(){
    WristMotor.set(0);
  }

  // Get wrist angle
  public double getWristAngle(){
    // *This should return an angle
    return WristEncoder.get() * ConversionConstants.AngleConversionValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
  }
}
