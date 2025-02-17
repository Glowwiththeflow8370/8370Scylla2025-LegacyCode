// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

@Logged
public class Intake extends SubsystemBase {
  

  private SparkFlex IntakeMotor;
  private SparkFlexConfig IntakeMotorConfig;


  /** Creates a new Intake. */
  public Intake() {
    IntakeMotor = new SparkFlex(IntakeConstants.IntakeMotor, MotorType.kBrushless);

    IntakeMotorConfig = new SparkFlexConfig();
    IntakeMotorConfig.idleMode(IdleMode.kBrake);
    
  }

  public void runIntake(double value){
    IntakeMotor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
