package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends Subsystem {
    public VictorSPX topMotor;
    public VictorSPX bottomMotor;
} 

public Feeder(int id) {
    topMotor1 = new VictorSPX(Constants.FEEDER_TOP_MOTORS[id]);
    bottomMotor1 = new VictorSPX(Constants.FEEDER_BOTTOM_MOTORS[id]);
}

@Override
public void periodic() {
    //switch statement??
}