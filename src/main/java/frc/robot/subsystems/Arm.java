// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ConversionConstants;

@Logged
public class Arm extends SubsystemBase{
  /** Creates TheTickler. */

  private SparkMax ArmMotor, ArmMotorFollower;
  private SparkMaxConfig ArmConfig, ArmFollowerConfig;
  private AbsoluteEncoder ArmEncoder;
  private DigitalInput ArmLimitSwitch;

  // This is the Coral/Ball intake
  public Arm() {
    ArmMotor = new SparkMax(ArmConstants.ArmMotor, MotorType.kBrushless);
    ArmMotorFollower = new SparkMax(ArmConstants.ArmMotorFollower, MotorType.kBrushless);

    // Right Intake Motor Config
    ArmConfig = new SparkMaxConfig();
    ArmConfig.idleMode(IdleMode.kBrake);
    ArmConfig.encoder
    .positionConversionFactor(ConversionConstants.AngleConversionValue);

    ArmFollowerConfig = new SparkMaxConfig();
    ArmFollowerConfig.idleMode(IdleMode.kBrake);
    ArmFollowerConfig.follow(ArmMotor, true);

    ArmMotor.configure(ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ArmMotorFollower.configure(ArmFollowerConfig, ResetMode.kResetSafeParameters, 
    PersistMode.kPersistParameters);
    
    ArmEncoder = ArmMotor.getAbsoluteEncoder();

    // Create Limit Switches
    ArmLimitSwitch = new DigitalInput(5);

  }

  public void RunArm(double speed){

    // add some logic here to stop the motor if it
    // hits a certain angle
    if(ArmLimitSwitch.get()){
      ArmMotor.set(0);
    }
    // else if(getArmAngle() >= Constants.ArmConstants.LowerBoundArmAngleCodeStop){

    // }
    else{
      ArmMotor.set(speed);
    }
    
  }

  public double getArmAngle(){
    return ArmEncoder.getPosition();
  }

  public void displayArmAngle(){
    System.out.println("Angle: " + ArmEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
