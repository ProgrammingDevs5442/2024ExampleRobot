// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoFieldOrient extends Command {

  Optional<Alliance> ally = DriverStation.getAlliance();

  /** Creates a new AutoFieldOrient. */
  public AutoFieldOrient() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.vision.hasTarget()) {
      if(ally.get() == Alliance.Blue) { // Set blue field orientation at the start of the match.
        RobotContainer.drivetrain.seedFieldRelative(
          new Pose2d(
            RobotContainer.odometry.getPose().getTranslation(),
            Rotation2d.fromDegrees(RobotContainer.vision.getFieldPose().getRotation().getDegrees())
          )
        );
      }
      else if(ally.get() == Alliance.Red) { // Set red field orientation at the start of the match.
        RobotContainer.drivetrain.seedFieldRelative(
          new Pose2d(
            RobotContainer.odometry.getPose().getTranslation(),
            Rotation2d.fromDegrees(180 + RobotContainer.vision.getFieldPose().getRotation().getDegrees())
          )
        );
      }

      RobotContainer.hasFieldOriented = true;
      RobotContainer.drivetrain.endRumble();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.hasFieldOriented;
  }
}
