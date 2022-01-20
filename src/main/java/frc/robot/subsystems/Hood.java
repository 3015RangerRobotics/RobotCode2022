// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;
    private double setPos = 0;
    private boolean holdAutoPos = false;
    private Timer timer = new Timer();


    public Hood(int id) {
        hoodMotor = new CANSparkMax(Constants.HOOD_MOTORS[id], MotorType.kBrushless);
    }
    //include the switch statement?
}