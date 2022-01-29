package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    public VictorSPX topMotor;
    public VictorSPX bottomMotor;
    public DigitalInput ballDetector;

    public Feeder(int id) {
        topMotor = new VictorSPX(Constants.FEEDER_TOP_MOTORS[id]);
        bottomMotor = new VictorSPX(Constants.FEEDER_BOTTOM_MOTORS[id]);
        ballDetector = new DigitalInput(Constants.FEEDER_BALL_DETECTORS[id]);
    }

    public boolean getBallDetector() {
        return ballDetector.get();
    }

    @Override
    public void periodic() {

    }

    public void setPercentOutput(double percent) {
        topMotor.set(ControlMode.PercentOutput, percent);
        bottomMotor.set(ControlMode.PercentOutput, percent);
    }
}