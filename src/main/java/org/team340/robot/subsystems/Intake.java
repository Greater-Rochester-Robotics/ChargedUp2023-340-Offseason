// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team340.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.math.Math2;
import org.team340.lib.subsystem.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.IntakeConstants;

public class Intake extends GRRSubsystem {

    private CANSparkMax upperMotor;
    private CANSparkMax lowerMotor;
    private TalonSRX innerMotor;

    /** Creates a new Intake. */
    public Intake() {
        super("Intake");
        upperMotor = createSparkMax("upperMotor", Constants.RobotMap.INTAKE_UPPER_MOTOR, MotorType.kBrushless);
        lowerMotor = createSparkMax("lowerMotor", Constants.RobotMap.INTAKE_LOWER_MOTOR, MotorType.kBrushless);
        innerMotor = new TalonSRX(Constants.RobotMap.INTAKE_INNER_MOTOR);

        upperMotor.enableVoltageCompensation(Constants.VOLTAGE);
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

        lowerMotor.enableVoltageCompensation(Constants.VOLTAGE);
        lowerMotor.setInverted(true);
        lowerMotor.setIdleMode(IdleMode.kBrake);
        lowerMotor.setClosedLoopRampRate(1);

        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 59424);
        lowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50952);

        innerMotor.enableVoltageCompensation(true);
        innerMotor.setInverted(true);
        innerMotor.setNeutralMode(NeutralMode.Brake);
        innerMotor.configContinuousCurrentLimit(10);
        innerMotor.configPeakCurrentLimit(15);

        upperMotor.burnFlash();
        lowerMotor.burnFlash();
    }

    public Command stopMotors() {
        return setMotors(0.0, 0.0);
    }

    public Command setMotors(double outerSpeed, double innerSpeed) {
        return setMotors(outerSpeed, outerSpeed, innerSpeed);
    }

    public Command setMotors(double upperSpeed, double lowerSpeed, double innerSpeed) {
        return commandBuilder(
            "intake.setMotors(" + Math2.toFixed(upperSpeed) + ", " + Math2.toFixed(lowerSpeed) + ", " + Math2.toFixed(innerSpeed) + ")"
        )
            .onInitialize(() -> {
                upperMotor.set(upperSpeed);
                lowerMotor.set(lowerSpeed);
                innerMotor.set(ControlMode.PercentOutput, innerSpeed);
            })
            .isFinished(true);
    }

    public Command pickUpCube() {
        return setMotors(IntakeConstants.OUTER_INTAKE_SPEED, IntakeConstants.INNER_INTAKE_SPEED);
    }
}
