// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.PathPlanner;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;

public class RobotContainer {

  public static PIDController speakerPID = shooterConstants.speakerPID; // PID loop for speaker auto rotate

  public static Pigeon2 pigeon2;
  public static TalonFX dFR, dFL, dBL, dBR;
  public static CANcoder cFR, cFL, cBL, cBR;


  // /* Setting up bindings for necessary control of the swerve drive platform */

  public final static CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final static SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric() //Field oriented drive
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final static SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric() //Field oriented drive
    .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage)
    .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Adds a deadzone to the robot's speed
    .withDriveRequestType(DriveRequestType.Velocity);



  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public final static Telemetry logger = new Telemetry(driveConstants.MaxSpeed);
  
  public static Vision vision = new Vision();
  public static Odometry odometry;
  public static PathPlanner pathplanner = new PathPlanner();

  public static boolean hasFieldOriented = false; // For auto-orienting at the start of TeleOp

  ///// OPERATOR CONTROLLER \\\\\
  public static XboxController xbox2 = new XboxController(1); // Operator joystick
  public static XboxController xbox1 = new XboxController(0);
  public static JoystickButton xbox2A = new JoystickButton(xbox2, 1);
  public static JoystickButton xbox2B = new JoystickButton(xbox2, 2);
  public static JoystickButton xbox2Y = new JoystickButton(xbox2, 4);


  public static Command ShootCargo, Shoot, Intake, Pancake, kShootOverride;



  private void configureBindings() {

    

    // /*-------------- CTRE CODE START ----------------*/

      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> 
              driveField
              .withVelocityX(Sine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));

      joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

      joystick.leftBumper().whileTrue( // ROBOT ORIENT WHILE HOLDING LEFT BUMPER
      drivetrain.applyRequest(() ->
          driveRobot
              // .withVelocityX(Math.pow(Deadzone(joystick.getLeftY()), 3) * driveConstants.MaxSpeed) //* (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive forward with negative Y (forward)
              // .withVelocityY(Math.pow(Deadzone(joystick.getLeftX()), 3) * driveConstants.MaxSpeed) //* (1 - (joystick.getLeftTriggerAxis() * 0.8))) // Drive left with negative X (left)
              .withVelocityX(Sine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(Cosine(joystick.getLeftX(), joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-Math.pow(Deadzone(joystick.getRightX()), 3) * driveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));

      joystick.leftBumper().onTrue(new Command() { // Reset driver rumble when left bumper is pressed
        @Override
        public void initialize() {
          xbox1.setRumble(RumbleType.kBothRumble, 0);
        }

        @Override
        public boolean isFinished() {
          return true;
        }
      });

      // reset the field-centric heading on down dpad press
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

      if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      }

      drivetrain.registerTelemetry(logger::telemeterize);

      


    // /*-------------- CTRE CODE END ----------------*/
    


    
  }
    
    
    //What we want
    // A Button - Fire command - shoot the funnyun if ready
    // Right Trigger - Rev button, aim the shooter if in speaker mode
    // (Speaker Mode / Amp Mode), two buttons to set it to either fire into the amp or the speaker.

  public static double Deadzone(double speed) {
    if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
    return 0;
  }

  public static double Cosine(double x, double y) {
    x = Deadzone(x);
    y = Deadzone(y);
    return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
  }

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

    dFR = new TalonFX(1);
    dFL = new TalonFX(3);
    dBL = new TalonFX(5);
    dBR = new TalonFX(7);

    cFR = new CANcoder(12);
    cFL = new CANcoder(9);
    cBL = new CANcoder(10);
    cBR = new CANcoder(11);

    ///// INPUTS \\\\\
    xbox2A = new JoystickButton(xbox2, 1);
    xbox2B = new JoystickButton(xbox2, 2);
    xbox2Y = new JoystickButton(xbox2, 4);
    
    ///// AUTO \\\\\
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
