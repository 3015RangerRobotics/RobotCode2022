// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAllTheWay extends SequentialCommandGroup {
  /** Creates a new ClimbAllTheWay. */
  public ClimbAllTheWay() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClimberHomePosition(),
        new WaitUntilCommand(RobotContainer.driverA::get),
        new ClimberToTop().withName("prepareToGrabBarTwo"),
        // new PrintCommand("a"),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToPullRobotUpToBarTwo"),
        // new PrintCommand("b"),
        new ClimberToBottom().withName("pullRobotUpToBarTwo"),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToReachToBarThree"),
        new FloppyArmUp(),//Constants.CLIMBER_FLOPPY_BAR_TWO_TO_THREE).withName("reachToBarThree"),
        new WaitCommand(1.5),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToGrabBarThree"),
        new ClimberToBarRelease(),
        new WaitUntilCommand(RobotContainer.driverA::get),
        new ClimberToTop().withName("grabBarThree"),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToPullUpToBarThree"),
        new ClimberToBottom().withName("pullUpToBarThree"),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToDropToGrabBarFour"),
        new FloppyArmDown(),//Constants.CLIMBER_FLOPPY_BAR_THREE_TO_FOUR).withName("dropToGrabBarFour"),
        new WaitCommand(1.5),
        new WaitUntilCommand(RobotContainer.driverA::get).withName("waitToGrabBarFour"),
        new ClimberToBarRelease().withName("grabBarFour"),
        new FloppyArmUp()
    );
  }
}
