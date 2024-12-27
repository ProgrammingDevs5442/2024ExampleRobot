// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.visionConstants;

/** Converts most of the important NetworkTables values to more friendly formats. */
public class CalculatedLimelight {

  ///// Values \\\\\
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  String key;



  public CalculatedLimelight(String key) {
    this.key = key;
  }



  public String getKey() {
    return key;
  }
  
  public NetworkTable getNetworkTable() {
    return inst.getTable(key);
  }

  public boolean hasTarget() {
    return getNetworkTable().getEntry("tid").getInteger(-1) >= 0;
  }

  public long getTargetID() {
    return getNetworkTable().getEntry("tid").getInteger(-1);
  }

  public double getTrust() {
    double[] targetRelative = getNetworkTable().getEntry("botpose_targetspace").getDoubleArray(new double[7]);

    return (1 / Math.sqrt((targetRelative[0] * targetRelative[0]) + (targetRelative[1] * targetRelative[1]) + (visionConstants.AngleDistrust * Math.sin(targetRelative[4] * (Math.PI / 180)))));
  }

  public Pose2d getFieldPose() {
    double[] fieldTable = getNetworkTable().getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

    return new Pose2d(
      new Translation2d(
        fieldTable[0], // X position (right/left from camera perspective)
        fieldTable[2]  // Z position (forward/back from camera perspective)
      ),
      Rotation2d.fromDegrees(fieldTable[5]) // Rotation (pitch)
    );
  }

  public Pose2d getTargetPose() {
    double[] fieldTable = getNetworkTable().getEntry("botpose_targetspace").getDoubleArray(new double[7]);

    return new Pose2d(
      new Translation2d(
        fieldTable[0], // X position (right/left from camera perspective)
        fieldTable[2]  // Z position (forward/back from camera perspective)
      ),
      Rotation2d.fromDegrees(fieldTable[5]) // Rotation (pitch)
    );
  }
}
