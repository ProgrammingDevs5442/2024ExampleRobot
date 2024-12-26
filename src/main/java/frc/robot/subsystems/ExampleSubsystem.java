// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  ///// Initialized Variables \\\\\
  double speed = 0;


  public ExampleSubsystem() {}

  // Method created to take in a speed value from ExampleCommand
  public void move(double speed) {
    this.speed = speed; // Copy the speed value from the command so other methods can use it here
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    RobotContainer.exampleMotor.set(speed); // Constantly update speed of motor (value from -1 to 1, as a percentage)
  }
}
