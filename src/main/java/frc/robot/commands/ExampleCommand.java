// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommand extends Command {
  /** Creates a new ExampleCommand. */

  ///// Initialized Variables \\\\\
  double speed = 0;
  

  public ExampleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.exampleSubsystem); // This connects the subsystem to the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = RobotContainer.Deadzone(RobotContainer.xbox2.getLeftY()); // Set the speed to up/down position on joystick

    RobotContainer.exampleSubsystem.move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
