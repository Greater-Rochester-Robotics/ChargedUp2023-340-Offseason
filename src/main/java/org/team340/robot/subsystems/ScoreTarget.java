// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.team340.lib.subsystem.GRRSubsystem;
import org.team340.robot.Constants.ArmConstants.Positions;
import org.team340.robot.Constants.IntakeConstants;

public class ScoreTarget extends GRRSubsystem {

    public ScoreTarget() {
        super("Score Target");
    }

    public enum Level {
        LOW,
        MID,
        HIGH,
        FAR,
    }

    private Level level = Level.FAR;

    public Command setLevel(Level level) {
        return runOnce(() -> this.level = level);
    }

    public Command getDriverCommand() {
        return new ProxyCommand(() -> {
            switch (level) {
                case LOW:
                    return intake.setMotors(IntakeConstants.SHOOT_SPEED_LOW, IntakeConstants.SHOOT_SPEED_INNER);
                case MID:
                    return intake.setMotors(IntakeConstants.SHOOT_SPEED_MID, IntakeConstants.SHOOT_SPEED_INNER);
                case HIGH:
                    return intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.SHOOT_SPEED_INNER);
                case FAR:
                    return intake.setMotors(
                        IntakeConstants.SHOOT_SPEED_FAR_UPPER,
                        IntakeConstants.SHOOT_SPEED_FAR_LOWER,
                        IntakeConstants.SHOOT_SPEED_INNER
                    );
                default:
                    return none();
            }
        }) {
            {
                addRequirements(arm);
            }
        };
    }

    public Command getCoDriverCommand() {
        return new ProxyCommand(() -> {
            Command command;

            switch (level) {
                case LOW:
                    command =
                        sequence(
                            intake.setMotors(IntakeConstants.SHOOT_SPEED_LOW, IntakeConstants.INNER_HOLD_SPEED),
                            arm.setPosition(Positions.SHOOT_LOW)
                        );
                    break;
                case MID:
                    command =
                        sequence(
                            intake.setMotors(IntakeConstants.SHOOT_SPEED_MID, IntakeConstants.INNER_HOLD_SPEED),
                            arm.setPosition(Positions.SHOOT_MID)
                        );
                    break;
                case HIGH:
                    command =
                        sequence(
                            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.INNER_HOLD_SPEED),
                            arm.setPosition(Positions.SHOOT_HIGH)
                        );
                    break;
                case FAR:
                    command =
                        sequence(
                            intake.setMotors(
                                IntakeConstants.SHOOT_SPEED_FAR_UPPER,
                                IntakeConstants.SHOOT_SPEED_FAR_LOWER,
                                IntakeConstants.INNER_HOLD_SPEED
                            ),
                            arm.setPosition(Positions.SHOOT_FAR)
                        );
                    break;
                default:
                    command = none();
                    break;
            }

            return command.finallyDo(interrupted -> intake.stopMotors());
        }) {
            {
                addRequirements(arm);
            }
        };
    }
}
