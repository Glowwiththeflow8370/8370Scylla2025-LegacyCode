// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForward extends Command {
  private Drivetrain drivetrain;
  private double startTime;
  private double endTime;
  private double speed;
  private double currentTime;
      
    /** Creates a new AutoDrive. */
  public DriveForward(Drivetrain drivetrain, double speed, double endTime) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.endTime = endTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // currentTime = Timer.getFPGATimestamp();
    System.out.println("Hallo");
    drivetrain.tank(-speed, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(currentTime - startTime < endTime){
    //   System.out.println("Running");
    //   return false;
    // }
    // System.out.println("Drive Command Done");
    // return true;

    return false;
  }
}
