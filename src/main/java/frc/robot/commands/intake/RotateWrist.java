// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateWrist extends Command {
  
  double angle;
  Wrist wrist;
  boolean reversed;
  
  /** Creates a new RotateWrist. */
  public RotateWrist(Wrist wrist, boolean reversed, double angle) {
    this.reversed = reversed;
    this.wrist = wrist;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Make this to rotate the wrist to a specified angle (Make sure it is flawless)
    if(reversed){
      wrist.rotateWrist(-IntakeConstants.WristRunValue);
    }
    else{
      wrist.rotateWrist(IntakeConstants.WristRunValue);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
