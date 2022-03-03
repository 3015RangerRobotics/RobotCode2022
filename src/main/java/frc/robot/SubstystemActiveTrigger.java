// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class SubstystemActiveTrigger extends Trigger {

    private SubsystemBase[] subsystems;

    public SubstystemActiveTrigger(SubsystemBase... subsystem) {
        this.subsystems = subsystem;

    }

    @Override
    public boolean get() {
        boolean output = false;
        for (SubsystemBase subsystem : subsystems) {
            output |= (subsystem.getCurrentCommand() != null);
        }
        return output;
    }
}
