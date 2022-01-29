package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    public VictorSPX topMotor;
    public VictorSPX bottomMotor; 
    //TODO: add DigitalInput which is the ball detaction sensor, make accessor functions for it.

    public Feeder(int id) {
        topMotor = new VictorSPX(Constants.FEEDER_TOP_MOTORS[id]);
        bottomMotor = new VictorSPX(Constants.FEEDER_BOTTOM_MOTORS[id]);
    }

    @Override
    public void periodic() {

    }
}