// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheTickler extends SubsystemBase {
  /** Creates a new TheTickler. */

  private SparkMax rightTicklerMotor, leftTicklerMotor;
  private SparkMaxConfig rightTicklerConfig, leftTicklerConfig;

  // This is the Coral/Ball intake
  public TheTickler() {
    rightTicklerMotor = new SparkMax(7, MotorType.kBrushless);
    leftTicklerMotor = new SparkMax(8, MotorType.kBrushless);

    // Right Intake Motor Config
    rightTicklerConfig = new SparkMaxConfig();

    // Left Intake Motor Config
    leftTicklerConfig = new SparkMaxConfig();
    

  }

  public void RunTickler(){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
