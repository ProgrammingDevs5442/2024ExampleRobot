// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.PathPlanner;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;

public class RobotContainer {

  /*---------------------------------------- VARIABLES ----------------------------------------*/


  ///// Xbox Controllers \\\\\

  // Main Driver
  public final static CommandXboxController joystick = new CommandXboxController(0); // Main Drive Controller (Commands)
  public static XboxController xbox1 = new XboxController(0); // Main Drive Controller (Buttons)
  
  // Operator
  public static XboxController xbox2 = new XboxController(1); // Operator Controller



  ///// Drivetrain Hardware \\\\\

  public static Pigeon2 pigeon2; // Gyroscope
  public static TalonFX dFR, dFL, dBL, dBR; // Drive Motors
  public static CANcoder cFR, cFL, cBL, cBR; // Drive Encoders


  
  ///// Drivetrain Setup \\\\\
  
  // Drivetrain
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  // Field Oriented Drive
  public final static SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage) // Speed deadzone
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Rotation speed deadzone
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Robot Oriented Drive
  public final static SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage) // Speed deadzone
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Rotation speed deadzone
    .withDriveRequestType(DriveRequestType.Velocity);

  // Emergency Brake Mode
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();



  ///// Position Tracking \\\\\

  // Telemetry - combines vision and odometry
  public final static Telemetry logger = new Telemetry(driveConstants.MaxSpeed);
  
  public static Vision vision = new Vision(); // Limelight position
  public static Odometry odometry; // Position based on wheel rotation

  // Autonomous Paths
  public static PathPlanner pathplanner = new PathPlanner();

  // Automatic Field Orienting
  public static boolean hasFieldOriented = false;





  private void configureBindings() {

    /*-------------------------- CTRE DRIVE CODE START --------------------------*/


    // Default drivetrain to Field Oriented
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> 
        driveField
        .withVelocityX(Sine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
            
    // Set drivetrain to Robot Oriented when holding left bumper
    joystick.leftBumper().whileTrue(
      drivetrain.applyRequest(() ->
        driveRobot
        .withVelocityX(Sine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    
    // Emergency Brake (wheels in X pattern)
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        

    /*--------------------------- CTRE DRIVE CODE END ---------------------------*/



    // Reset the Field Orientation on down dpad press
    joystick.povDown().onTrue(new Command() {
      @Override
      public void initialize() {
        drivetrain.seedFieldRelative();
        hasFieldOriented = true;
        xbox1.setRumble(RumbleType.kBothRumble, 0);
      }
      @Override 
      public boolean isFinished() {
        return true;
      }
    });

    // Reset driver controller rumble on left bumper press
    joystick.leftBumper().onTrue(new Command() {
      @Override
      public void initialize() {
        xbox1.setRumble(RumbleType.kBothRumble, 0);
      }
      
      @Override
      public boolean isFinished() {
        return true;
      }
    });

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

  }
    
  // Controller Deadzone
  public static double Deadzone(double speed) {
    if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
    return 0;
  }

  // Controller Y value curving
  public static double Cosine(double x, double y) {
    x = Deadzone(x);
    y = Deadzone(y);
    return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
  }

  // Controller X value curving
  public static double Sine(double x, double y) {
    x = Deadzone(x);
    y = Deadzone(y);
    return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.sin(Math.atan2(y,x));
  }

  public static void driveChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.setControl(
      driveRobot
      .withVelocityX(speeds.vxMetersPerSecond)
      .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond)
    );
  }

  public RobotContainer() {

    ///// Drivetrain Hardware \\\\\
    // Note - device IDs for actual drive code are set in TunerConstants.java
    // (the motors and encoders here are mostly for SmartDashboard)

    // Drive Motors
    dFR = new TalonFX(1);
    dFL = new TalonFX(3);
    dBL = new TalonFX(5);
    dBR = new TalonFX(7);

    // Drive Encoders (Rotation Angle)
    cFR = new CANcoder(12);
    cFL = new CANcoder(9);
    cBL = new CANcoder(10);
    cBR = new CANcoder(11);
    


    ///// AUTO \\\\\ TODO - Understand and document auto...
    pigeon2 = new Pigeon2(0);

    AutoBuilder.configureHolonomic(
      odometry::getPose,
      odometry::resetPose,
      odometry::getChassisSpeeds,
      RobotContainer::driveChassisSpeeds,
      driveConstants.config,
      
      () -> { //Flip the path if opposite team?
        return false;
      },
      RobotContainer.drivetrain
    );

    configureBindings();
  }

  public Command getAutonomousCommand() { 
    return pathplanner.getAuto();
  }
}
