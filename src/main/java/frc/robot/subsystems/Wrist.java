// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax wristMotor;
  private DigitalInput wristInnerLimitSwitch;
  private DigitalInput wristOuterLimitSwitch;
  
  /** Creates a new Wrist. */
  public Wrist() {
    // Setup the wrist.
    wristMotor = new CANSparkMax(Constants.RobotMap.WRIST_MOTOR, MotorType.kBrushless);
    
    // Wrist motor settings.
    wristMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    wristMotor.setInverted(true);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setClosedLoopRampRate(1);
    
    // Wrist frame settings.
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    wristInnerLimitSwitch = new DigitalInput(Constants.RobotMap.WRIST_INNER_LIMIT_SWITCH);
    wristOuterLimitSwitch = new DigitalInput(Constants.RobotMap.WRIST_OUTER_LIMIT_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
