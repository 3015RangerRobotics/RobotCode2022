package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor;
 
    public Climber() {
        climberMotor = new TalonFX(Constants.CLIMBER_MOTOR);
        climberMotor.configVoltageCompSaturation(12.5);
        climberMotor.enableVoltageCompensation(true);
        climberMotor.setSensorPhase(false);
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }
}