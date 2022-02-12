// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class DriveOneModule extends CommandBase {
  private int index;
  private double rotatePos = 0;

  /**
   * Creates a new DriveOneModule.
   * Drives a single module. The DPad controls angle and the Driver's left stick Y
   * axis controls speed.
   * 
   * @param index the index of the module to control.
   */
  public DriveOneModule(int index) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int hat = RobotContainer.getDriverDPad();
    if (hat != -1) {
      if (hat > 180) {
        rotatePos = hat - 360;
      } else {
        rotatePos = hat;
      }
    }

    SwerveModuleState[] states = RobotContainer.drive.getModuleStates();

    // Gotta love java line lengths
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

    RobotContainer.drive.driveOneModule(index, rotatePos, RobotContainer.getDriverLeftStickY(),
        ControlMode.PercentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
