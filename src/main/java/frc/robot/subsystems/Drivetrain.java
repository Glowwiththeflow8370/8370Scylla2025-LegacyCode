// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

// @Logged
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Motor Objects
  private TalonFX rightFront, rightBack, leftFront, leftBack;


  private StatusSignal<Angle> leftEncoder;
  private StatusSignal<Angle> rightEncoder;

  // Encoder Objects
  //private Encoder rightEncoder, leftEncoder;

  //NavX Gyro
  AHRS NavX;

  // Configuration objects
  private TalonFXConfiguration leftConfig, rightConfig;

  // Kinematics/Odometry Objects
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  // PID
  private PIDController leftPidController;
  private PIDController rightPidController;

  // Misc
  private DrivetrainConstants DriveConsts;
  private static RobotConfig config;

  private SysIdRoutine routine;

  double rightVelocityTranslational;
  double leftVelocityTranslational;

  public Drivetrain() {

    //------------------------------------------
    // Encoders (test them like this first, 
    // then figure out how to use w spark maxes)
    // They seem to work
    //------------------------------------------

    // NavX Gyro (Test All port configs to find which one the
    // NavX is connected to) * I think ours is fried lol
    NavX = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k4Hz);

    //--------
    // Motors
    //--------

    rightFront = new TalonFX(DrivetrainConstants.rightFrontPort);
    leftFront = new TalonFX(DrivetrainConstants.leftFrontPort);

    rightBack = new TalonFX(DrivetrainConstants.rightBackPort);
    leftBack = new TalonFX(DrivetrainConstants.leftBackPort);

    //----------------
    // Configurations
    //----------------
    
    // Create and set configurations (For left motor, I may have to specify the right motor)
    leftConfig = new TalonFXConfiguration();
    // Invert left motor
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = Constants.DrivetrainConstants.gearRatio;
    leftConfig.CurrentLimits.StatorCurrentLimit = 80;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 80;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    // Create and set configurations (For right motor)
    rightConfig = new TalonFXConfiguration();
    // Set the right motor to turn in the opposite direction of the left motor
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.Feedback.SensorToMechanismRatio = Constants.DrivetrainConstants.gearRatio;
    rightConfig.CurrentLimits.StatorCurrentLimit = 80;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 80;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightEncoder = rightFront.getPosition();
    leftEncoder = leftFront.getPosition();

    // Apply the configuration thing needed
    rightFront.getConfigurator().apply(rightConfig);
    leftFront.getConfigurator().apply(leftConfig);

    // Get the back motors to follow their respective front motors
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));
    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));

    // Sysid stuff
    routine = new SysIdRoutine(new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(null, null, this));


    // Reset Drivetrain Devices (NavX, KrakenEncoders, ect)
    resetEncoders();
    resetNavX();

    // Finish this by 2/8/25

    // Creating PID controllers (For use on the robot!)
    leftPidController = new PIDController(DrivetrainConstants.kP, 
    DrivetrainConstants.kI, DrivetrainConstants.kD);
    rightPidController = new PIDController(DrivetrainConstants.kP, 
    DrivetrainConstants.kI, DrivetrainConstants.kD);

    //----------------
    //   Kine/Odom
    //----------------

    // Fine tune these
    chassisSpeeds = new ChassisSpeeds(1.0, 0, 1.0);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.trackWidth));
    odometry = new DifferentialDriveOdometry(NavX.getRotation2d(), leftEncoder.getValueAsDouble(), rightEncoder.getValueAsDouble());
    DriveConsts = new DrivetrainConstants();
    config = DriveConsts.robotConfig;

    try {
      
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::tankRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            }
          return false;
                  },
          this // Reference to this subsystem to set requirements
    );

    } catch (NullPointerException e) {
        System.out.println("Failed to configure autobuilder");
    }

    // Referece code (Most likely wont be used since im pretty sure i know why the robot spins in circles)

  } // End of Constructor (Do not comment this bracket out)

  // Drive Command
  public void tank(double x, double y){
    // Multiplier in constants in case it is needed
    rightFront.set(x);
    leftFront.set(y);
  }

  // Get encoder Values
  public double getAverageEncoderValues(){
    return ((rightEncoder.getValueAsDouble() + (leftEncoder.getValueAsDouble()))/2.0); 
  }

  // Right encoder
  public double getRightEncValues(){
    return rightEncoder.getValueAsDouble();
  }

  // Left encoder
  public double getLeftEncValues(){
    return leftEncoder.getValueAsDouble();
  }

  // Get encoder values in terms of meters
  public double getLeftDistMeters(){
    return (leftEncoder.getValueAsDouble()) * DrivetrainConstants.metersPerCount;
  }

  public double getRightDistMeters(){
    return (rightEncoder.getValueAsDouble()) * DrivetrainConstants.metersPerCount;
  }

  public void updateEncoderPosition(){
    rightEncoder = rightFront.getPosition();
    leftEncoder = leftFront.getPosition();
  }

  // Get NavX encoder Values (The angle is offset by 7 inches)
  public double getAngle(){
    return NavX.getYaw();
  }

  public void thing(){
    NavX.getVelocityX();
  }

  public void resetNavX(){
    NavX.reset();
  }

  // Remember to figure out the setup of the
  // Commands below:
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d initialStartingPose){
      resetEncoders();
      odometry.resetPose(initialStartingPose);
  }
  // Reset motor positions (For encoders)
  public void resetEncoders(){
      rightFront.setPosition(0);
      leftFront.setPosition(0);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    // Fix this later
    return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, new Rotation2d(2.2));
  }

  // Tank Drive based on relative speeds
  public void tankRelative(ChassisSpeeds speeds){
    // Super close DONT GIVE UP!!!

    // Use these Comments as a reference, im quite surprised ai actually knows this (it speeds up
    // the process so i wont have to dig into countless github repos)

    // HAIL THE AI GODS, IT RUNS IN A LINE AND CRASHES INTO THE DRIVERSTATIONS!!!
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double leftOutput = leftPidController.calculate(getLeftEncValues(), wheelSpeeds.leftMetersPerSecond);
    double rightOutput = rightPidController.calculate(getRightEncValues(), wheelSpeeds.rightMetersPerSecond);

    //tank((rightOutput), (-leftOutput));
    tank((-rightOutput), (-leftOutput));
  }

  // Extra Debug commands
  public void displayEncoderValues(){
      System.out.println("Drivetrain Average Encoder Values: " + getAverageEncoderValues());
  }

  public void displayGyroValues(){
      System.out.println("Drivetrain Angle: " + getAngle());
  }

  // SysId commands (Could be Used)
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This needs to be converted somehow
    updateEncoderPosition();
    odometry.update(Rotation2d.fromDegrees(getAngle()), 
    new DifferentialDriveWheelPositions(getLeftDistMeters(), 
    getRightDistMeters()));

    // rightVelocityTranslational = DrivetrainConstants.wheelRadius * getRightEncValues();
    // leftVelocityTranslational = DrivetrainConstants.wheelRadius * getLeftEncValues();
    // Call the debug commands
    // displayEncoderValues();
    // displayGyroValues();
  }
}