package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor;
 
    public Climber() {
        climberMotor = new WPI_TalonFX(Constants.CLIMBER_MOTOR);
        //climberMotor.setInverted(false);
        climberMotor.configVoltageCompSaturation(12.5);
        climberMotor.enableVoltageCompensation(true);
        climberMotor.setSensorPhase(false);
    }