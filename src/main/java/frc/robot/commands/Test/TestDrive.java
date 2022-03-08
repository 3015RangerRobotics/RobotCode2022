// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class TestDrive extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");

  Drive drive;
  int stage = 0;
  boolean result = true;

  Timer timer;
  /** Creates a new TestDrive. */
  public TestDrive() {
    drive = RobotContainer.drive;
    addRequirements(drive);

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(stage) {
      case 0:
      case 1:
      case 2:
      case 3:
        drive.driveOneModule(stage, 0, 0, ControlMode.PercentOutput);
        stage++;
        break;
      case 4:
        if (timer.hasElapsed(1.5)) {
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 5:
        for(int i = 0; i < 4; i++) {
          result &= Math.abs(drive.getModuleRotation(i)) < 1.5;
        }
        testTable.getEntry("Drive module rotation test 1").setBoolean(result);
        result = true;
        stage++;
        break;
      case 6:
      case 7:
      case 8:
      case 9:
        drive.driveOneModule(stage - 6, 90, 0, ControlMode.PercentOutput);
        stage++;
        break;
      case 10:
        drive.resetIMU();
        timer.reset();
        timer.start();
        stage++;
        break;
      case 11:
        if (Math.abs(drive.getAngleDegrees()) < 0.5 && timer.hasElapsed(0.25)) {
          stage++;
        }
    }
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
