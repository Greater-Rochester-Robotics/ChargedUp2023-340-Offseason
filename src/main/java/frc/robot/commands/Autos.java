// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.commands.Routines.*;
import static frc.robot.RobotContainer.*;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.Positions;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
import frc.robot.commands.drive.util.DriveSetGyro;

/** Add your docs here. */
public class Autos {
    private static PIDController thetaController = new PIDController(SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_P, SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_I, SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_D);

    static {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command straight2Meters() {
        var path = PathPlanner.loadPathGroup("TestStraight2m", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command straight2MetersTurn() {
        var path = PathPlanner.loadPathGroup("TestStraight2mTurn", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command diagonal1Meter() {
        var path = PathPlanner.loadPathGroup("TestDiagonal1m", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command centerOnePiece (boolean balance) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Center1Piece" + (balance ? "Balance" : ""), SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, 0.0),
            arm.setPosition(Positions.SHOOT_HIGH).withTimeout(1.0),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.5),

            parallel(
                sequence(
                    new DriveFollowTrajectory(path.get(0), true),
                    balance ? new DriveBalanceRobot() : none()   
                ),
                sequence(
                    waitSeconds(0.25),
                    intake.stopMotors(),
                    storeCube()
                )
            )
        );
    }

    public static Command fivePiece() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("5Piece", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, 0.0),
            arm.setPosition(Positions.SHOOT_HIGH).withTimeout(0.6),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.SHOOT_SPEED_INNER),
            
            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    waitSeconds(0.7),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(2)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(3)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            deadline(
                new DriveFollowTrajectory(path.get(4)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    storeCube()
                )
            )
        );
    }

    public static Command fourPiece(boolean balance) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("4Piece" + (balance ? "Balance" : ""), SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, 0.0),
            arm.setPosition(Positions.SHOOT_HIGH).withTimeout(0.6),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.SHOOT_SPEED_INNER),
            
            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    waitSeconds(0.7),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(2)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            deadline(
                new DriveFollowTrajectory(path.get(3)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    storeCube()
                )
            ),

            balance ? new DriveBalanceRobot() : none()
        );
    }

    public static Command threePiece(boolean balance) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("3Piece" + (balance ? "Balance" : ""), SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, 0.0),
            arm.setPosition(Positions.SHOOT_HIGH).withTimeout(0.6),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH, IntakeConstants.SHOOT_SPEED_INNER),
            
            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    waitSeconds(0.7),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            deadline(
                new DriveFollowTrajectory(path.get(2)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    storeCube()
                )
            ),

            balance ? new DriveBalanceRobot() : none()
        );
    }

    public static Command bumpFourPiece (boolean balance) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Bump4Piece" + (balance ? "Balance" : ""), SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, 0.0),
            arm.setPosition(Positions.SHOOT_FAR).withTimeout(0.6),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),

            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(2)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            deadline(
                new DriveFollowTrajectory(path.get(3)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    storeCube()
                )
            ),

            balance ? new DriveBalanceRobot() : none()
        );
    }

    public static Command bumpThreePiece (boolean balance) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Bump3Piece" + (balance ? "Balance" : ""), SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.PATH_MAXIMUM_ACCELERATION);

        return sequence(
            new DriveSetGyro(180),

            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, 0.0),
            arm.setPosition(Positions.SHOOT_FAR).withTimeout(0.6),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),

            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    deadline(
                        waitSeconds(2.0),
                        intake(true)
                    ),
                    intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.INNER_HOLD_SPEED),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR_UPPER, IntakeConstants.SHOOT_SPEED_FAR_LOWER, IntakeConstants.SHOOT_SPEED_INNER),
            waitSeconds(0.15),

            deadline(
                new DriveFollowTrajectory(path.get(2)),
                sequence(
                    waitSeconds(0.2),
                    intake.stopMotors(),
                    storeCube()
                )
            ),

            balance ? new DriveBalanceRobot() : none()
        );
    }
}
