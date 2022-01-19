package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

 
public class Intake extends Subsystem {
    private VictorSP intakeMotor;

    private final double INTAKE_SPEED = 1; //update number later. should this be in constants?
    private final double OUTTAKE_SPEED = -1; //update number later. should this be in constants?

	public Intake(int id) {
		intakeMotor = new VictorSP(Constants.INTAKE_MOTORS(id));
	}

    @Override
	public void initDefaultCommand() {
    
    public void intakeIntake() {
		intakeLeft.set(INTAKE_SPEED);
        intakeRight.set(INTAKE_SPEED);
	}

	public void intakeOuttake() {
		intakeLeft.set(OUTTAKE_SPEED);
        intakeRight.set(OUTTAKE_SPEED);
	}

	public void intakeStop() {
		intakeLeft.set(0);
        intakeRight.set(0);
	}
	}
}