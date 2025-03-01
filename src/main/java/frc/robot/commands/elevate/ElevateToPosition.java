// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevateToPosition extends Command {
  /** Creates a new ElevateToPosition. */

  Elevator elevator;
  double position;

  public ElevateToPosition(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      elevator.moveMotor(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(elevator.getEncoderValues() >= position){
      System.out.println("Command Finished");
      System.out.println("Elevator Stopped");
      elevator.moveMotor(0);
      return true;
    }
    System.out.println("Command Running");
    return false;
  }
}
