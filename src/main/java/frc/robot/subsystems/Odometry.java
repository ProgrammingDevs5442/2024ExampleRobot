// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

public class Odometry extends SubsystemBase {

  // Position of wheels in 2D space
  private static Translation2d FR = new Translation2d(driveConstants.ChassisModulePosX, driveConstants.ChassisModulePosY); // +x +y
  private static Translation2d FL = new Translation2d(-driveConstants.ChassisModulePosX, driveConstants.ChassisModulePosY); // -x +y
  private static Translation2d BL = new Translation2d(-driveConstants.ChassisModulePosX, -driveConstants.ChassisModulePosY); // -x -y
  private static Translation2d BR = new Translation2d(driveConstants.ChassisModulePosX, -driveConstants.ChassisModulePosY); // +x -y

  // Inital position of swerve modules
  private static SwerveModulePosition FRpos;
  private static SwerveModulePosition FLpos;
  private static SwerveModulePosition BLpos;
  private static SwerveModulePosition BRpos;
  private static SwerveModulePosition[] swerveModulePositions;

  // Swerve drive object
  private static SwerveDriveKinematics kinematics;

   // Encoder and gyro mesurements to estimate position
  public static SwerveDrivePoseEstimator poseEstimator;

  // 2D coordinates of the playing field and robot
  private static Pose2d pose = new Pose2d(); // Represents robot position
  private final Field2d m_field = new Field2d(); // Represents field and robot position on it


  /** Creates a new Odometry. */
  public Odometry() {

    SmartDashboard.putData("Field", m_field); // TODO comment here


    // Finding initial positon of the motors
    FRpos = RobotContainer.drivetrain.getModule(1).getPosition(true);
    FLpos = RobotContainer.drivetrain.getModule(0).getPosition(true);
    BLpos = RobotContainer.drivetrain.getModule(2).getPosition(true);
    BRpos = RobotContainer.drivetrain.getModule(3).getPosition(true);

    // Initial object setup
    swerveModulePositions = new SwerveModulePosition[] {FRpos, FLpos, BLpos, BRpos};

    kinematics = new SwerveDriveKinematics(FR,FL,BL,BR);

    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      RobotContainer.drivetrain.getRotation3d().toRotation2d(),
      swerveModulePositions,
      new Pose2d(),
      VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2)), // Drive Standard Deviation (Default 0.01 and 5deg)
      VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30)) // Vision Standard Deviation (Default 0.02 and 30deg)
    );

  }

  public Pose2d getPose() {
    return pose;
  }

  // Reset position
  public void resetPose(Pose2d pose) {
    System.out.println("Reset Pose");
    poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation3d().toRotation2d(), swerveModulePositions, pose);
    RobotContainer.drivetrain.seedFieldRelative(pose);
  }

  // Gets speed of the chassis to know how far the robot has moved
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
      RobotContainer.drivetrain.getModule(1).getCurrentState(), // FR
      RobotContainer.drivetrain.getModule(0).getCurrentState(), // FL
      RobotContainer.drivetrain.getModule(2).getCurrentState(), // BL
      RobotContainer.drivetrain.getModule(3).getCurrentState()  // BR
    );
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Constantly update wheel positions
    FRpos = RobotContainer.drivetrain.getModule(1).getPosition(true); 
    FLpos = RobotContainer.drivetrain.getModule(0).getPosition(true);
    BLpos = RobotContainer.drivetrain.getModule(2).getPosition(true);
    BRpos = RobotContainer.drivetrain.getModule(3).getPosition(true);

    swerveModulePositions = new SwerveModulePosition[] {FRpos, FLpos, BLpos, BRpos};

    pose = poseEstimator.update(RobotContainer.drivetrain.getRotation3d().toRotation2d(), swerveModulePositions);  

    if (RobotContainer.vision.hasTarget()) { // Should run if any tags are in view
      poseEstimator.addVisionMeasurement(RobotContainer.vision.getFieldPose(), Timer.getFPGATimestamp());
    }

    // Displaying values on SmartDashboard
    SmartDashboard.putNumber("Robot Position X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Position Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Position Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());


    // Update robot positon on field
    m_field.setRobotPose(poseEstimator.getEstimatedPosition()); // TODO Check if we can move this above SmartDashboard
  }
  
}