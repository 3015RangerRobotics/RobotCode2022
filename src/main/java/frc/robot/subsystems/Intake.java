package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 
public class Intake extends SubsystemBase {
    private VictorSPX intakeMotor;

    private final double INTAKE_SPEED = 1; //update number later. should this be in constants?
    private final double OUTTAKE_SPEED = -1; //update number later. should this be in constants?

	public Intake(int id) {
		intakeMotor = new VictorSPX(Constants.INTAKE_MOTORS[id]);
	}

    public void intakeIntake() {
		intakeMotor.set(ControlMode.PercentOutput, INTAKE_SPEED);
	}

	public void intakeOuttake() {
		intakeMotor.set(ControlMode.PercentOutput, OUTTAKE_SPEED);
	}

	public void intakeStop() {
		intakeMotor.set(ControlMode.PercentOutput, 0);
	}
}