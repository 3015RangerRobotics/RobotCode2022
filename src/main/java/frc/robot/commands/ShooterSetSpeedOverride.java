// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.FloatSerializer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterSetSpeedOverride extends CommandBase {
  /** Creates a new ShooterSetSpeedOverride. */
  public ShooterSetSpeedOverride() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter[0].setMinSpeedOverride(true);
    RobotContainer.shooter[0].stop();
    RobotContainer.shooter[1].setMinSpeedOverride(true);
    RobotContainer.shooter[1].stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter[0].setMinSpeedOverride(false);
    RobotContainer.shooter[1].setMinSpeedOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
