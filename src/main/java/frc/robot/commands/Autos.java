// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.climbing.ForwardClimb;
import frc.robot.commands.climbing.ReversClimb;

public final class Autos {
  // All autonomous sequences will be added here

  /** Example static factory for an autonomous command. */
  public static Command AutoDriveCommand() {
    return new DriveForward(Robot.Drivetrain, 1, 2);
  }

  public static Command BasicAutoScore(){
    return new BasicAutoScore();
  }

  // public static Command HoldClimb(){
  //   return new ForwardClimb(Robot.Climb);
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
