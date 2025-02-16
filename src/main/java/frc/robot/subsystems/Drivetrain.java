// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
//import frc.robot.commands.Drive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Motor Objects
  private TalonFX rightFront, rightBack, leftFront, leftBack;

  // Encoder Objects
  private Encoder rightEncoder, leftEncoder;

  //NavX Gyro
  AHRS NavX;

  // Emergency Gyro (In case NavX does not want to work)
  //private ADXRS450_Gyro gyro;

  // Configuration objects
  private TalonFXConfiguration leftConfig, rightConfig;

  // Kinematics/Odometry Objects
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  // Misc
  private DrivetrainConstants DriveConsts;
  private static RobotConfig config;

  public Drivetrain() {

    //------------------------------------------
    // Encoders (test them like this first, 
    // then figure out how to use w spark maxes)
    // They seem to work
    //------------------------------------------
    
    // Right encoder
    rightEncoder = new Encoder(DrivetrainConstants.rightEncoderChanA, 
    DrivetrainConstants.rightEncoderChanB,false,EncodingType.k4X);
    // Left encoder
    leftEncoder = new Encoder(DrivetrainConstants.leftEncoderChanA,
    DrivetrainConstants.leftEncoderChanB,false,EncodingType.k4X);

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

    // Create and set configurations (For right motor)
    rightConfig = new TalonFXConfiguration();
    // Set the right motor to turn in the opposite direction of the left motor
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply the configuration thing needed
    rightFront.getConfigurator().apply(rightConfig);
    leftFront.getConfigurator().apply(leftConfig);

    // Get the back motors to follow their respective front motors
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));
    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));

    // Finish this by 2/8/25

    //----------------
    //   Kine/Odom
    //----------------

    // Fine tune these
    chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.trackWidth));
    odometry = new DifferentialDriveOdometry(NavX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
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

  } // End of Constructor (Do not comment this bracket out)

  // Drive Command
  public void tank(double x, double y){
    // Multiplier in constants in case it is needed
    rightFront.set(x);
    leftFront.set(y);
  }

  // Get encoder Values
  public double getAverageEncoderValues(){
    return ((rightEncoder.getDistance() + leftEncoder.getDistance())/2.0); 
  }

  // Get NavX encoder Values
  public double getAngle(){
    return NavX.getAngle();
  }

  // Remember to figure out the setup of the
  // Commands below:
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d initialStartingPose){
      odometry.resetPose(initialStartingPose);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    // Fix this later
    return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, new Rotation2d(2.2));
  }

  // Tank Drive based on relative speeds
  public void tankRelative(ChassisSpeeds speeds){
    // Debug it for now
    System.out.println("Running");
  }

  // Extra Debug commands
  public void displayEncoderValues(){
      System.out.println("Drivetrain Average Encoder Values: " + getAverageEncoderValues());
  }

  public void displayGyroValues(){
      System.out.println("Drivetrain Angle: " + getAngle());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Call the debug commands
    displayEncoderValues();
    displayGyroValues();
  }
}