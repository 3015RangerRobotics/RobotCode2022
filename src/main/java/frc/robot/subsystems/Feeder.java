package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    public VictorSPX topMotor;
    public VictorSPX bottomMotor;
    public DigitalInput ballDetector;
    boolean doPeriodic = false;
    private double id;

    /**
     * Do not use
     */
    @Deprecated
    public Feeder() {

    }

    public Feeder(int id) {
        topMotor = new VictorSPX(Constants.FEEDER_TOP_MOTORS[id]);
        ballDetector = new DigitalInput(Constants.FEEDER_BALL_DETECTORS[id]);
        this.id = id;
        doPeriodic = true;
    }

    public boolean getBallDetector() {
        return ballDetector.get();
    }

    @Override
    public void periodic() {
        if (doPeriodic) {
            SmartDashboard.putBoolean(String.format("Ball Detector $d", id), getBallDetector());
        }
    }

    public void setPercentOutput(double percent) {
        topMotor.set(ControlMode.PercentOutput, percent);
    }
}