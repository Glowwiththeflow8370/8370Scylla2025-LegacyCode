// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;

public class Drive extends Command {
  // Object Declaration
  private Drivetrain drivetrain;
  private CommandPS4Controller ps4Controller;

  double xAxis;
  double yAxis;

  boolean reversed = false;

  /** Creates a new Drive. */
  public Drive(Drivetrain drivetrain, CommandPS4Controller ps4Controller) {
    this.drivetrain = drivetrain;
    this.ps4Controller = ps4Controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xAxis = MathUtil.applyDeadband(ps4Controller.getRightX(), DrivetrainConstants.Deadband);
    yAxis = MathUtil.applyDeadband(ps4Controller.getLeftY(), DrivetrainConstants.Deadband);
    drivetrain.tank((xAxis - yAxis), (xAxis + yAxis));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
