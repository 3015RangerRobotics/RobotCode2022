// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Zeros the gyro based on the robots current facing  or set a specific angle to zero based on current facing */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveZeroGyro extends CommandBase {
  Timer timer = new Timer();
  double angle;

  public DriveZeroGyro() {
    this(0);
  }

  public DriveZeroGyro(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.drive.setIMU(angle);
  }

  public boolean isFinished() {
    return timer.get() > 0.05;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
