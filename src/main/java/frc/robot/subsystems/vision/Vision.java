// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Telemetry;

public class Vision extends SubsystemBase {

  ArrayList<CalculatedLimelight> limelights = new ArrayList<CalculatedLimelight>(); // List of Limelights for Field-Oriented calculation
  
  // Initialize Calculated Limelights (custom object)
  CalculatedLimelight frontLL = new CalculatedLimelight("limelight");
  CalculatedLimelight backLL = new CalculatedLimelight("limelight-main");

  Telemetry logger;

  /** Creates a new Vision. */
  public Vision() {
    // Add Calculated Limelights to list
    limelights.add(frontLL);
    limelights.add(backLL);

    logger = RobotContainer.logger;
  }

  /** Returns true if any limelights see a target. */
  public boolean hasTarget() {
    for (int l = 0; l < limelights.size(); l++) {
      if (limelights.get(l).hasTarget()) return true; // If any of the limelights see a target, return true.
    }
    return false; // If none of the limelights see a target, return false.
  }

  /** Returns the ID of the currently visible target for the specified limelight (or -1 if there are none). */
  public long getTargetID(int limelightID) {
    return limelights.get(limelightID).getTargetID();
  }

  /** Returns the target-relative Pose2d of the specified limelight. */
  public Pose2d getTargetPose(int limelightID) {
    return limelights.get(limelightID).getTargetPose();
  }

  /** Returns the field-relative Pose2d calculated from all limelights. */
  public Pose2d getFieldPose() {
    double fX = 0;
    double fY = 0;
    double fR = 0;
    double tot = 0;

    // Weighted Average of all limelights, based on trust (how accurate we think one is at a given point)
    for (int l = 0; l < limelights.size(); l++) {
      if (limelights.get(l).hasTarget()) {
        fX += limelights.get(l).getFieldPose().getX() * limelights.get(l).getTrust(); // Weighted X pos
        fY += limelights.get(l).getFieldPose().getY() * limelights.get(l).getTrust(); // Weighted Y pos
        fR += limelights.get(l).getFieldPose().getRotation().getDegrees() * limelights.get(l).getTrust(); // Weighted rotation
        tot += limelights.get(l).getTrust(); // Add the weights to divide later
      }
    }

    fX /= tot; // Final weighted average of X pos
    fY /= tot; // Final weighted average of Y pos
    fR /= tot; // Final weighted average of rotation

    return new Pose2d(
      new Translation2d(
        fX,
        fY
      ),
      Rotation2d.fromDegrees(fR)
    );
  }



  @Override
  public void periodic() {
    Optional<Alliance> ally = DriverStation.getAlliance();

    SmartDashboard.putNumber("Robot Field Position X", getFieldPose().getX());
    SmartDashboard.putNumber("Robot Field Position Y", getFieldPose().getY());
    SmartDashboard.putBoolean("Has Target", hasTarget());
  }
}
