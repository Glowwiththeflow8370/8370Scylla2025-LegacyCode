// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CompConsts;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.climbing.ForwardClimb;
import frc.robot.commands.climbing.ClimbStop;
import frc.robot.commands.climbing.ReversClimb;

import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
// @Logged
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
  
  // To L1
  

  // RotateWristToPosition sourceAngle = new RotateWristToPosition(Robot.Wrist, 250);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  // private final CommandJoystick m_Joystick = 
  //     new CommandJoystick(OperatorConstants.kStickControllerPort);
  SendableChooser<Command> AutoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default commands of the subsystems
    Robot.Drivetrain.setDefaultCommand(new Drive(Robot.Drivetrain, m_driverController));
    Robot.Climb.setDefaultCommand(climbStop);
    

    // Setup Named Commands
    NamedCommands.registerCommand("ElevateToL1", climbStop);
    // NamedCommands.registerCommand("EjectCoral", forwardIntake);

    // Configure Data that is sent to the Dashboard (PathPlannerPaths)
    AutoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    (stream) -> CompConsts.isCompetiton
      ? stream.filter(auto -> auto.getName().startsWith("comp"))
      : stream
  );
  // Auto fallback solutions (use if pathplanner is not working)
  

  SmartDashboard.putData("Choices", AutoChooser);
    
    // Configure the trigger bindings
    configureBindings();
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
    m_driverController.cross().whileTrue(reversClimb);
    // m_driverController.circle().whileTrue(Autos.HoldClimb().withTimeout(0.5));

    // Elevator Buttons
    // m_driverController.pov(180).whileTrue(elevatorDown);
    // m_driverController.pov(0).whileTrue(elevatorUp);

    // m_driverController.pov(270).onTrue(sourceSetPosition);
    // m_driverController.pov(90).onTrue(sourceAngle);
    // Arm Buttons
    // m_driverController.L1().whileTrue(armUp);
    // m_driverController.R1().whileTrue(armDown);

    // Intake/Arm buttons
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
