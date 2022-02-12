// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class DriveFeedbackWhileDisabled extends CommandBase {
  /** Creates a new DriveFeedbackWhileDisabled. */
  public DriveFeedbackWhileDisabled() {
    addRequirements(RobotContainer.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SwerveModuleState[] states = RobotContainer.drive.getModuleStates();

    SmartDashboard.putNumber("Front Right Swerve Module Angle",
    states[Drive.ModuleIndices.kFrontRight.index].angle.getDegrees());
SmartDashboard.putNumber("Front Left Swerve Module Angle",
    states[Drive.ModuleIndices.kFrontLeft.index].angle.getDegrees());
SmartDashboard.putNumber("Back Left Swerve Module Angle",
    states[Drive.ModuleIndices.kBackLeft.index].angle.getDegrees());
SmartDashboard.putNumber("Back Right Swerve Module Angle",
    states[Drive.ModuleIndices.kBackRight.index].angle.getDegrees());

SmartDashboard.putNumber("Front Right Swerve Module Velocity",
    states[Drive.ModuleIndices.kFrontRight.index].speedMetersPerSecond);
SmartDashboard.putNumber("Front Left Swerve Module Velocity",
    states[Drive.ModuleIndices.kFrontLeft.index].speedMetersPerSecond);
SmartDashboard.putNumber("Back Left Swerve Module Velocity",
    states[Drive.ModuleIndices.kBackLeft.index].speedMetersPerSecond);
SmartDashboard.putNumber("Back Right Swerve Module Velocity",
    states[Drive.ModuleIndices.kBackRight.index].speedMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
