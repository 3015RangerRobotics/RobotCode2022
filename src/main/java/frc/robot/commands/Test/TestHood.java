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
import frc.robot.subsystems.Hood;

public class TestHood extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");
  
  Hood hood;
  int stage;
  int[] testAngles = {0, 10, 15, 5, 20};
  boolean result = true;

  Timer timer;

  /** Creates a new TestHood. */
  public TestHood() {
    hood = RobotContainer.hood;
    addRequirements(hood);

    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testTable.getEntry("test stage").setNumber(stage);
    switch(stage) {
      case 0: /* start homing hood */
        if (hood.getReverseLimit()) {
          stage++;
          break;
        }
        hood.setHoodOutputPercentage(-1);
        hood.setReverseLimit(false);
        stage++;
        break;
      case 1: /* home hood, immediately end test if this fails */
        if (hood.getReverseLimit()) {
          hood.setHoodOutputPercentage(0);
          testTable.getEntry("Hood homing test").setBoolean(true);
          hood.setReverseLimit(true);
          hood.resetZero();
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(1.5)) {
          testTable.getEntry("Hood homing test").setBoolean(false);
          hood.setHoodOutputPercentage(0);
          stage = 8;
        }
        break;
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        hood.setHoodPosition(testAngles[stage - 2]);
        if (timer.hasElapsed(0.75)) {
          result &= Math.abs(hood.getHoodPosition() - testAngles[stage - 2]) < 0.25;
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 7:
        testTable.getEntry("Hood position test").setBoolean(result);
        stage++;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 8;
  }
}
