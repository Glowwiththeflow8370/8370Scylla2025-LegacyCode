// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CompConsts;
//import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
//import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.climbing.ForwardClimb;
import frc.robot.commands.climbing.ClimbStop;
import frc.robot.commands.climbing.ReversClimb;
import frc.robot.commands.elevate.ElevatorDown;
import frc.robot.commands.elevate.ElevatorStop;
import frc.robot.commands.elevate.ElevatorUp;

import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Camera Stuff goes here?
  // UsbCamera usbTestCam = new UsbCamera("DriverCam", 0);
  // MjpegServer server1 = new MjpegServer("DriverCamServer", 1181);
  // UsbCamera cam = CameraServer.addCamera();

  // Climb Commands
  ForwardClimb climbCommand = new ForwardClimb(Robot.Climb);
  ReversClimb reversClimb = new ReversClimb(Robot.Climb);
  ClimbStop climbStop = new ClimbStop(Robot.Climb);

  // Elevator Commands
  ElevatorUp elevatorUp = new ElevatorUp(Robot.Elevator);
  ElevatorDown elevatorDown = new ElevatorDown(Robot.Elevator);
  ElevatorStop elevatorStop = new ElevatorStop(Robot.Elevator);

  // If needed, this is where the pathplanner autos will go

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    (stream) -> CompConsts.isCompetiton
      ? stream.filter(auto -> auto.getName().startsWith("comp"))
      : stream
  );
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Robot.Drivetrain.setDefaultCommand(new Drive(Robot.Drivetrain, m_driverController));

    // Good to know this works!
    AutoChooser.setDefaultOption("Basic Drive", Autos.AutoDriveCommand());
    //AutoChooser.addOption("Placeholder", null);
    SmartDashboard.putData("Choices", AutoChooser);

    // Configure the trigger bindings
    configureBindings();

    // Set the default commands of the subsystems
    Robot.Climb.setDefaultCommand(climbStop);
    Robot.Elevator.setDefaultCommand(elevatorStop);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Climb Buttons
    m_driverController.triangle().whileTrue(climbCommand);
    m_driverController.circle().whileTrue(reversClimb);

    // Elevator Buttons
    m_driverController.square().whileTrue(elevatorUp);
    m_driverController.cross().whileTrue(elevatorDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoChooser.getSelected();
  }
}
