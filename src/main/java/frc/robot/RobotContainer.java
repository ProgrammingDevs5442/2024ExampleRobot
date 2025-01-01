// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.PathPlanner;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.ExampleCommand;

public class RobotContainer {

  /*---------------------------------------- VARIABLES ----------------------------------------*/

  ///// Xbox Controllers \\\\\
  // Main Driver
  public final static CommandXboxController joystick = new CommandXboxController(0); // Main Drive Controller (Commands)
  public static XboxController xbox1 = new XboxController(0); // Main Drive Controller (Buttons)
  
  // Operator
  public static XboxController xbox2 = new XboxController(1); // Operator Controller


  ///// Example \\\\\  
  // Example Hardware
  public static TalonFX exampleMotor;

  // Example Subsystem
  public static ExampleSubsystem exampleSubsystem = new ExampleSubsystem();


  ///// Drivetrain \\\\\
  // Drivetrain Hardware
  public static Pigeon2 pigeon2; // Gyroscope
  public static TalonFX dFR, dFL, dBL, dBR; // Drive Motors
  public static CANcoder cFR, cFL, cBL, cBR; // Drive Encoders
  
  // Drivetrain Setup  
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  public final static SwerveRequest.FieldCentric driveField = CommandSwerveDrivetrain.driveField; // Field Oriented
  public final static SwerveRequest.RobotCentric driveRobot = CommandSwerveDrivetrain.driveRobot; // Robot Oriented
  public final static SwerveRequest.RobotCentric driveAuto = CommandSwerveDrivetrain.driveAuto;   // Robot Oriented (Auto)

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // Emergency brake (x-shape)


  ///// Position Tracking \\\\\

  // Telemetry - combines vision and odometry
  public final static Telemetry logger = new Telemetry(driveConstants.MaxSpeed);
  
  public static Vision vision = new Vision(); // Limelight position
  public static Odometry odometry; // Position based on wheel rotation

  // Autonomous Paths
  public static PathPlanner pathplanner = new PathPlanner();
  public static boolean hasFieldOriented = false; // Used to automatically set orientation at start of match



  /** Place to configure bindings for the CommandXboxController, such as driving modes. */
  private void configureBindings() {

    // DEFAULT: Field Oriented (command executed periodically)
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> driveField));
    
    joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveRobot)); // Robot Oriented while holding left bumper

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // Emergency Brake (wheels in X pattern) when A button pressed
    joystick.povDown().onTrue(drivetrain.resetField()); // Reset Field orientation on down dpad press
    joystick.leftBumper().onTrue(drivetrain.endRumble()); // End controller rumble on left bumper press


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

  }  



  /** Function that returns a given speed, as long as it is above the deadzone set in Constants. */
  public static double Deadzone(double speed) {
    if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
    return 0;
  }



  /** Drive with given speeds during Autonomous. */
  public static void driveChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.setControl(
      driveAuto
      .withVelocityX(speeds.vxMetersPerSecond)
      .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond)
    );
  }



  public RobotContainer() {

    ///// Example Hardware \\\\\
    exampleMotor = new TalonFX(32); // Make sure to update the ID to match an actual motor
    exampleSubsystem.setDefaultCommand(new ExampleCommand()); // Connect the Command and Subsystem


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
