// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Drive up the charge station until balanced.
 * Stops moving when robot is coming to a balance.
 * Driver has lateral control if specified.
 */
public class DriveBalanceRobot extends CommandBase {
    private static final double SPEED_IN_PER_SEC = 20.0;
    private static final double POS_THRES_DEG = 4.0;
    private static final double VEL_THRES_DEG_PER_SEC = 7.0;
    private static final double STOPPED_FIN_MIN_TIME = 0.75;

    private final Timer timer = new Timer();

    private double lastAngleDegrees;
    private double lastTime;
    private double lastStoppedFinalTimestamp;

    /**
     * Creates a new DriveBalance command.
     */
    public DriveBalanceRobot() {
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize () {
        lastAngleDegrees = Double.POSITIVE_INFINITY;
        lastTime = -1;
        lastStoppedFinalTimestamp = -1000.0;
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute () {
        double angleDegrees =
            swerveDrive.getGyroRotation2d().getCos() * swerveDrive.getGyroInDegPitch() + swerveDrive.getGyroRotation2d().getSin() * swerveDrive.getGyroInDegRoll();

        double time = timer.get();
        double angleVelocityDegreesPerSec = lastAngleDegrees > 1000.0 ? 0.0 : (time - lastTime) * (lastAngleDegrees - angleDegrees);

        lastTime = time;
        lastAngleDegrees = angleDegrees;

        boolean shouldStopTemporary = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > VEL_THRES_DEG_PER_SEC) || (angleDegrees > 0.0 && angleVelocityDegreesPerSec < -VEL_THRES_DEG_PER_SEC);

        if (Math.abs(angleDegrees) < POS_THRES_DEG) lastStoppedFinalTimestamp = time;

        if (shouldStopTemporary) {
            swerveDrive.driveRobotCentric(new ChassisSpeeds(), false, false);
        } else if (time - lastStoppedFinalTimestamp < STOPPED_FIN_MIN_TIME) {
            swerveDrive.driveX();
        } else {
            swerveDrive.driveFieldRelative(
                Units.inchesToMeters(SPEED_IN_PER_SEC) * (angleDegrees > 0.0 ? 1.0 : -1.0),
                0.0,
                0.0,
                true
            );
        }
    }

    @Override
    public void end (boolean interrupted) {
        swerveDrive.driveRobotCentric(new ChassisSpeeds(), false, false);
    }

    @Override
    public boolean isFinished () {
        return (DriverStation.getMatchTime() >= 0.0 && DriverStation.getMatchTime() < 0.25) || Robot.robotContainer.driverMoving();
    }
}