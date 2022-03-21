// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class SubsystemActiveTrigger extends Trigger {
    private SubsystemBase[] subsystems;
    boolean alwaysActive = false;

    /**
     * A trigger that can be used to see if any input subsystems 
     * are running a command. Good for locking out 
     * buttons/triggers when a subsystem is active.
     * 
     * @param subsystem subsystems whose active state are to be polled
     */
    public SubsystemActiveTrigger(SubsystemBase... subsystem) {
        this.subsystems = subsystem;

    }

    @Override
    public boolean get() {
        //start with a false output
        boolean output = false;
        //for each subsystem...
        for (SubsystemBase subsystem : subsystems) {
            //find whether it is running, and if true or it with output
            output |= (subsystem.getCurrentCommand() != null);
        }
        //returns true if any subsystem in subsystems is running a command
        return output || alwaysActive;
    }

    public void setAlwaysActive(boolean active) {
        this.alwaysActive = active;
    }
}
