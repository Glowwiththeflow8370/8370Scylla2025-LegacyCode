// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevate.ElevateToPosition;
import frc.robot.commands.intake.RunIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAutoScore extends SequentialCommandGroup {
  /** Creates a new BasicAutoScore. */
  public BasicAutoScore() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForward(Robot.Drivetrain, 1, 1).withTimeout(0.5),
    new ElevateToPosition(Robot.Elevator, 1300), new RunIntake(Robot.Intake, true).withTimeout(2));
  }
}
