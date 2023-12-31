// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team340.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.team340.lib.subsystem.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.ArmConstants;

public class Arm extends GRRSubsystem {

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;
    private SparkMaxPIDController armController;

    private CANSparkMax wristMotor;
    private AbsoluteEncoder wristEncoder;
    private SparkMaxPIDController wristController;

    private double lastArmPosition, lastWristPosition;
    private boolean positionInitialized;

    private double armSetpoint = 0.0;
    private double wristSetpoint = 0.0;

    /** Creates a new Arm. */
    public Arm() {
        super("Arm");
        armMotor = createSparkMax("armMotor", Constants.RobotMap.ARM_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkMaxAbsoluteEncoder("armEncoder", armMotor, Type.kDutyCycle);
        armController = armMotor.getPIDController();

        // Arm motor settings.
        armMotor.enableVoltageCompensation(Constants.VOLTAGE);
        armMotor.setInverted(false);
        armMotor.setIdleMode(IdleMode.kBrake);

        // Arm frame settings.
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

        // Arm encoder settings.
        armEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR);
        armEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR / 60);
        armEncoder.setInverted(true);
        armEncoder.setZeroOffset(0.880642);

        armController.setOutputRange(-ArmConstants.ARM_MAX_PID_OUTPUT, ArmConstants.ARM_MAX_PID_OUTPUT);

        armController.setFeedbackDevice(armEncoder);
        armController.setP(ArmConstants.ARM_P);
        armController.setI(ArmConstants.ARM_I);
        armController.setD(ArmConstants.ARM_D);
        armController.setFF(ArmConstants.ARM_F);

        wristMotor = createSparkMax("wristMotor", Constants.RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        wristEncoder = createSparkMaxAbsoluteEncoder("wristEncoder", wristMotor, Type.kDutyCycle);
        wristController = wristMotor.getPIDController();
        wristController.setPositionPIDWrappingEnabled(false);

        // Wrist motor settings.
        wristMotor.enableVoltageCompensation(Constants.VOLTAGE);
        wristMotor.setInverted(false);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setClosedLoopRampRate(0.4);

        // Wrist frame settings.
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

        // Wrist encoder settings
        wristEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR);
        wristEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR / 60);
        wristEncoder.setZeroOffset(3.33883683);

        wristController.setFeedbackDevice(wristEncoder);
        wristController.setP(ArmConstants.WRIST_P);
        wristController.setI(ArmConstants.WRIST_I);
        wristController.setD(ArmConstants.WRIST_D);
        wristController.setFF(ArmConstants.WRIST_F);

        wristController.setOutputRange(-ArmConstants.WRIST_MAX_PID_OUTPUT, ArmConstants.WRIST_MAX_PID_OUTPUT);

        armMotor.burnFlash();
        wristMotor.burnFlash();
    }

    @Override
    public void periodic() {
        // smartDashboardDelay++;
        // if(smartDashboardDelay >= 50){
        SmartDashboard.putNumber("armEncoder", armEncoder.getPosition());
        SmartDashboard.putNumber("wristEncoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("wristCorrectedAngle", getWristCorrectedAngle());
        SmartDashboard.putNumber("applied wrist", wristMotor.getAppliedOutput());
        SmartDashboard.putNumber("applied arm", armMotor.getAppliedOutput());
        SmartDashboard.putNumber("armSetpoint", armSetpoint);
        SmartDashboard.putNumber("wristSetpoint", wristSetpoint);
        // smartDashboardDelay = 0;
        // }

    }

    public void setWristBrake(boolean isBrakeMode) {
        wristMotor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    // arm methods

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    // wrist methods

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    private double getWristFeedForward() {
        return 0.0; // Math.cos(getWristCorrectedAngle()) * ArmConstants.WRIST_GRAV_FF;
    }

    /**
     * returns the angle of the center of mass of the wrist with respect to the parallel of the ground
     * @return
     */
    private double getWristCorrectedAngle() {
        return wristEncoder.getPosition() + armEncoder.getPosition() - 3.949;
    }

    // Arm Commands

    public Command setPosition(Position position) {
        if (
            position.getArm() < ArmConstants.ARM_MIN ||
            position.getArm() > ArmConstants.ARM_MAX ||
            position.getWrist() < ArmConstants.WRIST_MIN ||
            position.getWrist() > ArmConstants.WRIST_MAX
        ) {
            DriverStation.reportWarning("Arm position out of range", false);
            return Commands.runOnce(() -> {}, this);
        }

        return commandBuilder("arm.setPosition()")
            .onInitialize(() -> {})
            .onExecute(() -> {
                armController.setReference(position.getArm(), CANSparkMax.ControlType.kPosition);
                wristController.setReference(position.getWrist(), CANSparkMax.ControlType.kPosition, 0, getWristFeedForward());

                armSetpoint = position.getArm();
                wristSetpoint = position.getWrist();

                lastArmPosition = getArmPosition();
                lastWristPosition = getWristPosition();
                positionInitialized = true;
            })
            .onEnd(interrupted -> {
                armMotor.stopMotor();
                wristMotor.stopMotor();
                if (!interrupted) {
                    lastArmPosition = position.getArm();
                    lastWristPosition = position.getWrist();
                }
            })
            .isFinished(() -> {
                return (
                    Math.abs(getArmPosition() - position.getArm()) < ArmConstants.ARM_TOLERANCE &&
                    Math.abs(getWristPosition() - position.getWrist()) < ArmConstants.WRIST_TOLERANCE
                );
            });
    }

    public Command holdPosition() {
        return commandBuilder("arm.holdPosition()")
            .onInitialize(() -> {})
            .onExecute(() -> {
                if (positionInitialized) {
                    armController.setReference(lastArmPosition, CANSparkMax.ControlType.kPosition);
                    wristController.setReference(lastWristPosition, CANSparkMax.ControlType.kPosition, 0, getWristFeedForward());

                    armSetpoint = lastArmPosition;
                    wristSetpoint = lastWristPosition;
                }
            })
            .onEnd(interrupted -> {
                armMotor.stopMotor();
                wristMotor.stopMotor();
            });
    }

    public Command setDutyCycle(Supplier<Double> armDutyCycle, Supplier<Double> wristDutyCycle) {
        return commandBuilder("arm.setDutyCycle()")
            .onInitialize(() -> {})
            .onExecute(() -> {
                armMotor.set(armDutyCycle.get());
                wristMotor.set(wristDutyCycle.get());

                lastArmPosition = getArmPosition();
                lastWristPosition = getWristPosition();
                positionInitialized = true;
            })
            .onEnd(() -> {
                armMotor.stopMotor();
                wristMotor.stopMotor();
            });
    }

    public static class Position {

        private final double arm;
        private final double wrist;

        public Position(double arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }

        public double getArm() {
            return arm;
        }

        public double getWrist() {
            return wrist;
        }
    }
}
