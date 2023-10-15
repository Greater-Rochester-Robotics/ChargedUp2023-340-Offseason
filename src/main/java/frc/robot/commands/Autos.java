// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.commands.Routines.*;

import java.util.List;

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.Positions;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.subsystems.Arm;

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
        var path = PathPlanner.loadPathGroup("Straight2m", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command straight2MetersTurn() {
        var path = PathPlanner.loadPathGroup("Straight2mTurn", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command diagonal1Meter() {
        var path = PathPlanner.loadPathGroup("Diagonal1m", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
        return sequence(
            new DriveResetGyroToZero(),
            new DriveFollowTrajectory(path.get(0), true)
        );
    }

    public static Command fivePiece() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("5Piece", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
        System.out.println(path.size());
        return sequence(
            new DriveSetGyro(180),
            shootHigh().withTimeout(0.7),
            parallel(
                new DriveFollowTrajectory(path.get(0), true),
                sequence(
                    waitSeconds(1.0),
                    intake().withTimeout(1.5),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR + 0.1, IntakeConstants.SHOOT_SPEED_FAR),
            waitSeconds(0.25),
            parallel(
                new DriveFollowTrajectory(path.get(1)),
                sequence(
                    intake().withTimeout(1.5),
                    arm.setPosition(Positions.SHOOT_FAR)
                )
            ),
            intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR + 0.1, IntakeConstants.SHOOT_SPEED_FAR).withTimeout(0.25)
        );
    }
}