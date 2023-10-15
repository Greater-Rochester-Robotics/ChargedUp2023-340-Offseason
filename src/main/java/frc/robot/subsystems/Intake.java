// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.CommandBuilder;

public class Intake extends SubsystemBase {
  private byte smartDashboardDelay = 0;

  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;
  private DigitalInput cubeLimit;

  /** Creates a new Intake. */
  public Intake() {
    upperMotor = new CANSparkMax(Constants.RobotMap.INTAKE_UPPER_MOTOR, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(Constants.RobotMap.INTAKE_LOWER_MOTOR, MotorType.kBrushless);
    
    upperMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    upperMotor.setInverted(false);
    upperMotor.setIdleMode(IdleMode.kBrake);
    upperMotor.setClosedLoopRampRate(1);

    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 59424);
    upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50952);

    lowerMotor.restoreFactoryDefaults();
    lowerMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    lowerMotor.setInverted(false);
    lowerMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setClosedLoopRampRate(1);
    
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 59424);
    lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50952);

    cubeLimit = new DigitalInput(Constants.RobotMap.INTAKE_CUBE_LIMIT_DIGITAL_INPUT);

    upperMotor.burnFlash();
    lowerMotor.burnFlash();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    smartDashboardDelay++;
    if(smartDashboardDelay >= 50){
       SmartDashboard.putBoolean("cube limit switch", hasCube());
       smartDashboardDelay = 0;
    }
  }

  public boolean hasCube(){
    return !cubeLimit.get();
  }

    //----------Commands----------

  public Command setMotors(double upperSpeed, double lowerSpeed) {
    return new CommandBuilder(this)
      .onInitialize(() -> {
          upperMotor.set(upperSpeed);
          lowerMotor.set(lowerSpeed);
        }
      )
      .isFinished(true);
  }

  public Command setMotors(double speed) {
    return setMotors(speed, speed);
  }

  public Command pickUpCube() {
    return setMotors(IntakeConstants.INTAKE_SPEED)
      .andThen(Commands.waitUntil(this::hasCube))
      .andThen(Commands.waitSeconds(0.05))
      .andThen(setMotors(0.0));
  }
}
