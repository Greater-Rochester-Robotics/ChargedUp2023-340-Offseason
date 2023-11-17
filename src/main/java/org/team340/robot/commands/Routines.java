package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Constants.ArmConstants.Positions;
import org.team340.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Routines {

    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command intake() {
        return parallel(intake.pickUpCube(), arm.setPosition(Positions.INTAKE));
    }

    public static Command storeCube() {
        return sequence(intake.setMotors(0.0, IntakeConstants.INNER_HOLD_SPEED), arm.setPosition(Positions.SAFE));
    }

    public static Command shootSlow() {
        return sequence(
            intake.setMotors(IntakeConstants.SHOOT_SPEED_SLOW, IntakeConstants.INNER_HOLD_SPEED),
            arm.setPosition(Positions.SHOOT_SLOW).withTimeout(1.5),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_SLOW, IntakeConstants.SHOOT_SPEED_INNER)
        );
    }

    public static Command shootNormal() {
        return sequence(
            intake.setMotors(IntakeConstants.SHOOT_SPEED_NORMAL, IntakeConstants.INNER_HOLD_SPEED),
            arm.setPosition(Positions.SHOOT_NORMAL).withTimeout(1.5),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_NORMAL, IntakeConstants.SHOOT_SPEED_INNER)
        );
    }

    public static Command shootFast() {
        return sequence(
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAST, IntakeConstants.INNER_HOLD_SPEED),
            arm.setPosition(Positions.SHOOT_FAST).withTimeout(1.5),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAST, IntakeConstants.SHOOT_SPEED_INNER)
        );
    }

    public static Command shootVeryFast() {
        return sequence(
            intake.setMotors(IntakeConstants.SHOOT_SPEED_VERY_FAST, IntakeConstants.INNER_HOLD_SPEED),
            arm.setPosition(Positions.SHOOT_VERY_FAST).withTimeout(1.5),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_VERY_FAST, IntakeConstants.SHOOT_SPEED_INNER)
        );
    }

    public static Command stopShooting() {
        return sequence(intake.stopMotors(), arm.setPosition(Positions.SAFE));
    }
}
