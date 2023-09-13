// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;
  private DigitalInput cubeLimit;

  /** Creates a new Intake. */
  public Intake() {
    upperMotor = new CANSparkMax(Constants.RobotMap.INTAKE_UPPER_MOTOR, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(Constants.RobotMap.INTAKE_LOWER_MOTOR, MotorType.kBrushless);
    
    upperMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    upperMotor.setInverted(true);
    upperMotor.setIdleMode(IdleMode.kBrake);
    upperMotor.setClosedLoopRampRate(1);

    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    lowerMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    lowerMotor.setInverted(true);
    lowerMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setClosedLoopRampRate(1);
    
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    cubeLimit = new DigitalInput(Constants.RobotMap.INTAKE_CUBE_LIMIT_DIGITAL_INPUT);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
