package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreolib.ChoreoSwerveControllerCommand;
import choreolib.ChoreoTrajectory;
import choreolib.TrajectoryManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.SwerveBase;
import org.team340.robot.Constants.SwerveConstants;

/**
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final ProfiledPIDController rotController = new ProfiledPIDController(
        SwerveConstants.ROTATION_PID.p(),
        SwerveConstants.ROTATION_PID.i(),
        SwerveConstants.ROTATION_PID.d(),
        SwerveConstants.ROTATION_CONSTRAINTS
    );

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    /**
     * Zeroes the IMU to a specified yaw.
     */
    public Command zero(double yaw) {
        return runOnce(() -> zeroIMU(yaw)).withName("swerve.zero(" + Math2.toFixed(yaw) + ")");
    }

    /**
     * Drives the robot as a percent of its max velocity (inputs are from {@code -1.0} to {@code 1.0}).
     * @param x X speed.
     * @param y Y speed.
     * @param rot Rotational speed.
     * @param fieldRelative If the robot should drive field relative.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, boolean fieldRelative) {
        return commandBuilder("swerve.drive()")
            .onExecute(() -> driveSpeeds(speedsFactory.percent(x.get(), y.get(), rot.get(), fieldRelative)));
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked at a field relative angle.
     * @param x X speed.
     * @param y Y speed.
     * @param angle The desired field relative angle to point at in radians.
     */
    public Command driveAngle(Supplier<Double> x, Supplier<Double> y, double angle) {
        return commandBuilder("swerve.driveAngle(" + Math2.toFixed(angle) + ")")
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveSpeeds(speedsFactory.angle(x.get(), y.get(), angle, rotController)));
    }

    /**
     * Drives the robot while snapping to facing up or down the field, whichever is closer.
     * Ends when the robot is at the determined angle.
     * @param x X speed.
     * @param y Y speed.
     */
    public Command driveSnap180(Supplier<Double> x, Supplier<Double> y) {
        return either(driveAngle(x, y, 0.0), driveAngle(x, y, Math.PI), () -> Math.abs(MathUtil.angleModulus(imu.getYaw())) < Math2.HALF_PI)
            .withName("swerve.driveSnap180()");
    }

    /**
     * Drives the modules into an X formation to prevent the robot from moving.
     */
    public Command lock() {
        return commandBuilder("swerve.lock()").onExecute(this::lockWheels);
    }

    public Command followTrajectory(String trajFile) {
        return followTrajectory(trajFile, false, false);
    }

    public Command followTrajectory(String trajFile, boolean resetPose, boolean stopOnEnd) {
        ChoreoTrajectory traj = TrajectoryManager.getInstance().getTrajectory(trajFile + ".json");
        return sequence(
            resetPose
                ? runOnce(() -> {
                    Pose2d initialPose = traj.getInitialPose(true);
                    zeroIMU(initialPose.getRotation().getRadians());
                    resetOdometry(initialPose);
                })
                : none(),
            new ChoreoSwerveControllerCommand(
                traj,
                this::getPosition,
                new PIDController(SwerveConstants.AUTO_XY_PID.p(), SwerveConstants.AUTO_XY_PID.i(), SwerveConstants.AUTO_XY_PID.d()),
                new PIDController(SwerveConstants.AUTO_XY_PID.p(), SwerveConstants.AUTO_XY_PID.i(), SwerveConstants.AUTO_XY_PID.d()),
                new PIDController(
                    SwerveConstants.AUTO_ROTATION_PID.p(),
                    SwerveConstants.AUTO_ROTATION_PID.i(),
                    SwerveConstants.AUTO_ROTATION_PID.d()
                ),
                speeds -> driveSpeeds(speeds, true, false),
                true,
                this
            ),
            stopOnEnd ? runOnce(this::stop) : none()
        );
    }
}
