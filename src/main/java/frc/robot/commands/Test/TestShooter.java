// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class TestShooter extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");

  Shooter leftShooter;
  Shooter rightShooter;
  int stage = 0;
  double lowTestRPM = 2000;
  double highTestRPM = 4000;
  Timer timer;
  boolean result = true;

  /** Creates a new TestShooter. */
  public TestShooter() {
    leftShooter = RobotContainer.shooter[0];
    rightShooter = RobotContainer.shooter[1];

    addRequirements(leftShooter, rightShooter);

    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    timer.reset();
    timer.start();
    testTable.getEntry("Left Shooter low speed test").setBoolean(false);
    testTable.getEntry("Left Shooter high speed test").setBoolean(false);
    testTable.getEntry("Right Shooter low speed test").setBoolean(false);
    testTable.getEntry("Right Shooter high speed test").setBoolean(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testTable.getEntry("test stage").setNumber(stage);
    switch(stage) {
      case 0: /* set left shooter to low rpm */
        leftShooter.setRPM(lowTestRPM);
        stage++;
        break;
      case 1: /* wait until primed or 2 seconds */
        if(leftShooter.isPrimed()) {
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(2)) {
          timer.reset();
          timer.start();
          testTable.getEntry("Left Shooter low speed test").setBoolean(false);
          stage += 2;
        }
        break;
      case 2: /* hold low speed for one second */
        if (timer.hasElapsed(0.4)) {
          result &= leftShooter.isPrimed();
        }
        if (timer.hasElapsed(1.5)) {
          testTable.getEntry("Left Shooter low speed test").setBoolean(result);
          result = true;
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 3: /* set to high speed */
        leftShooter.setRPM(highTestRPM);
        stage++;
      case 4: /* wait until primed or 2 seconds */
        if (leftShooter.isPrimed()) {
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(2)) {
          timer.reset();
          timer.start();
          leftShooter.stop();
          testTable.getEntry("Left Shooter high speed test").setBoolean(false);
          stage += 2;
        }
        break;
      case 5: /* hold high speed for one second */
        if (timer.hasElapsed(0.4)) {
          result &= leftShooter.isPrimed();
        }
        if (timer.hasElapsed(1.5)) {
          testTable.getEntry("Left Shooter high speed test").setBoolean(result);
          result = true;
          timer.reset();
          timer.start();
          leftShooter.stop();
          stage++;
        }
        break;
      case 6: /* set right shooter to low rpm */
        rightShooter.setRPM(lowTestRPM);
        stage++;
        break;
      case 7: /* wait until primed or 2 seconds */
        if(rightShooter.isPrimed()) {
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(2)) {
          timer.reset();
          timer.start();
          testTable.getEntry("Right Shooter low speed test").setBoolean(false);
          stage += 2;
        }
        break;
      case 8: /* hold low speed for one second */
        if (timer.hasElapsed(0.4)) {
          result &= rightShooter.isPrimed();
        }
        if (timer.hasElapsed(1.5)) {
          testTable.getEntry("Right Shooter low speed test").setBoolean(result);
          result = true;
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 9: /* set to high speed */
        rightShooter.setRPM(highTestRPM);
        stage++;
      case 10: /* wait until primed or 2 seconds */
        if (rightShooter.isPrimed()) {
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(2)) {
          timer.reset();
          timer.start();
          rightShooter.stop();
          testTable.getEntry("Right Shooter high speed test").setBoolean(false);
          stage += 2;
        }
        break;
      case 11: /* hold high speed for one second */
        if (timer.hasElapsed(0.4)) {
          result &= rightShooter.isPrimed();
        }
        if (timer.hasElapsed(1.5)) {
          testTable.getEntry("Right Shooter high speed test").setBoolean(false);
          result = true;
          timer.reset();
          timer.start();
          rightShooter.stop();
          stage++;
        }
        break;
        
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 12;
  }
}
