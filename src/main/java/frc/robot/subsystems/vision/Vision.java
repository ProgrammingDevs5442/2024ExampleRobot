// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.visionConstants;
import frc.robot.subsystems.Telemetry;

public class Vision extends SubsystemBase {

  boolean target, targetMain;
  double latency;

  private Pose2d fieldPose, localPose, localPoseMain;

  public long targetID, targetIDMain;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable, visionTableMain;

  Telemetry logger;

  /** Creates a new Vision. */
  public Vision() {
    visionTable = inst.getTable("limelight");
    visionTableMain = inst.getTable("limelight-main");

    fieldPose = new Pose2d();
    localPose = new Pose2d();
    logger = RobotContainer.logger;
  }

  public boolean target() {
    return this.target;
  }

  public Pose2d getFieldPose() {
    return this.fieldPose;
  }

  public Pose2d getLocalPose() {
    return this.localPose;
  }

  public double getLatency() {
    return this.latency;
  }
  
  public boolean isFacingSpeaker() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get() == Alliance.Blue && (targetID == 7 || targetID == 8)) return true; // If on Blu and speaker tags in view
    else if(ally.get() == Alliance.Red && (targetID == 3 || targetID == 4)) return true; // If on Red and speaker tags in view
   
    return false; // Otherwise, return false
  }


  @Override
  public void periodic() {
    targetID = visionTable.getEntry("tid").getInteger(-1); // Get ID of current April Tag
    targetIDMain = visionTableMain.getEntry("tid").getInteger(-1);

    // target = visionTable.getEntry("tv").getBoolean(false);
    // target = visionTable.getEntry("tv").getDouble(0) == 1;
    target = (targetID >= 0); // Does it see a target?
    // targetMain = visionTableMain.getEntry("tv").getBoolean(false);
    // targetMain = visionTableMain.getEntry("tv").getDouble(0) == 1;
    targetMain = (targetIDMain >= 0); // Does it see a target?

    Optional<Alliance> ally = DriverStation.getAlliance();
    // if(ally.get() == Alliance.Blue) visionTable.getEntry("priorityid").setInteger(7); // If on Blu and speaker tags in view TODO - FIX PRIORITY
    // else if(ally.get() == Alliance.Red) visionTable.getEntry("priorityid").setInteger(4); //  Red and speaker tags in view

    //Get robot relative positional data

    double[] targetRelative = visionTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    
    this.localPose = new Pose2d(
      new Translation2d(
        targetRelative[0],
        targetRelative[2]
      ),
      Rotation2d.fromDegrees(targetRelative[5])
    );
    
    double[] targetRelativeMain = visionTableMain.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    
    this.localPoseMain = new Pose2d(
      new Translation2d(
        targetRelativeMain[0],
        targetRelativeMain[2]
      ),
      Rotation2d.fromDegrees(targetRelative[5])
    );
    // this.target = (visionTable.getEntry("tv").getDouble(0) == 1);


    //Get field relative positional data

    double[] fieldRelative = visionTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    double[] fieldRelativeMain = visionTableMain.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

    double fieldRelativeX, fieldRelativeY, fieldRelativeR, trust, trustMain;
    
    
    if (target && targetMain) { // If both cameras see targets, average the positions read by both.
      // fieldRelativeX = (fieldRelative[0] + fieldRelativeMain[0]) / 2;
      // fieldRelativeY = (fieldRelative[1] + fieldRelativeMain[1]) / 2;
      // fieldRelativeR = (fieldRelative[5] + fieldRelativeMain[5]) / 2;
      
      trust = (1 / Math.sqrt((targetRelative[0] * targetRelative[0]) + (targetRelative[1] * targetRelative[1]) + (visionConstants.AngleDistrust * Math.sin(targetRelative[4] * (Math.PI / 180)))));
      trustMain = (1 / Math.sqrt((targetRelativeMain[0] * targetRelativeMain[0]) + (targetRelativeMain[1] * targetRelativeMain[1]) + (visionConstants.AngleDistrust * Math.sin(targetRelativeMain[4] * (Math.PI / 180)))));
      
      fieldRelativeX = ((fieldRelative[0] * trust) + (fieldRelativeMain[0] * trustMain)) / (trust + trustMain);
      fieldRelativeY = ((fieldRelative[1] * trust) + (fieldRelativeMain[1] * trustMain)) / (trust + trustMain);
      fieldRelativeR = ((fieldRelative[5] * trust) + (fieldRelativeMain[5] * trustMain)) / (trust + trustMain);
    
    }
    else if (target) { // If first camera sees a target, only use that position.
      fieldRelativeX = fieldRelative[0];
      fieldRelativeY = fieldRelative[1];
      fieldRelativeR = fieldRelative[5];
    }
    else if (targetMain) { // If second camera sees a target, only use that position.
      fieldRelativeX = fieldRelativeMain[0];
      fieldRelativeY = fieldRelativeMain[1];
      fieldRelativeR = fieldRelativeMain[5];
    }
    else { // Otherwise, do nothing.  TODO - MAKE SURE THIS DOESN'T CAUSE ISSUES!
      fieldRelativeX = 0;
      fieldRelativeY = 0;
      fieldRelativeR = 0;
    }

    this.fieldPose = new Pose2d(
      new Translation2d(
        fieldRelativeX,
        fieldRelativeY
      ),
      Rotation2d.fromDegrees(fieldRelativeR)
    );
    

    this.latency = fieldRelative[6];

    SmartDashboard.putNumber("Robot Field Position X", fieldRelativeX);
    SmartDashboard.putNumber("Robot Field Position Y", fieldRelativeY);
    SmartDashboard.putBoolean("Target", target);
    SmartDashboard.putBoolean("Target Main", targetMain);
  }
}
