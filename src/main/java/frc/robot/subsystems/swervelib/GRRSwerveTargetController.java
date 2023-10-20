package frc.robot.subsystems.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.Constants;
import frc.robot.GRRMath;

// TODO Investigate undesirable behavior while entering and leaving stopped targets with rotational motion

/**
 * Controller that calculates module states based on prior targets while compensating for kinematic constraints as well
 * as globally determining module optimization to shorten and prevent unpredictable behavior during heading transition periods.
 */
public class GRRSwerveTargetController {

    /**
     * A target.
     */
    public static class GRRSwerveTarget {

        /**
         * The target's chassis speeds.
         */
        public final ChassisSpeeds chassisSpeeds;
        /**
         * The target's module states.
         */
        public final SwerveModuleState[] moduleStates;

        /**
         * Create a target.
         * @param chassisSpeeds The target's chassis speeds.
         * @param moduleStates The target's module states.
         */
        public GRRSwerveTarget(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
            this.chassisSpeeds = chassisSpeeds;
            this.moduleStates = moduleStates;
        }
    }

    /**
     * The kinematics instance in use.
     */
    private final SwerveDriveKinematics kinematics;
    /**
     * Swerve module positions relative to the center of the robot (in meters).
     */
    private final Translation2d[] modulePositions;

    /**
     * Create the target controller.
     * @param config The general config.
     * @param kinematics The kinematics instance in use.
     * @param modulePositions Swerve module positions relative to the center of the robot (in meters).
     */
    public GRRSwerveTargetController(SwerveDriveKinematics kinematics, Translation2d[] modulePositions) {
        this.kinematics = kinematics;
        this.modulePositions = modulePositions;
    }

    /**
     * Calculates a new target.
     * @param lastTarget The last target.
     * @param desiredSpeeds The new desired chassis speeds.
     * @return The new target.
     */
    public GRRSwerveTarget calculate(GRRSwerveTarget lastTarget, ChassisSpeeds desiredSpeeds) {
        // - Determine the desired swerve module states.
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);

        // - Preliminary wheel speed desaturation based on the configured max robot velocity.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY);

        // - Determine desired speeds based on desaturated module states.
        desiredSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);

        // - Declare a list for modules that require special behavior for their
        //   heading outside of the kinematics calculated heading.
        List<Optional<Rotation2d>> overrideHeading = new ArrayList<>(modulePositions.length);

        // - Declare a tripwire to determine the robot is stopping. If this changes
        //   to true, turn s will not be calculated.
        boolean stopping = false;

        // (Special behavior for module rotation when the desired chassis speeds is 0)
        // If the desired target has no movement:
        if (
            GRRMath.twist2dEpsilonEquals(
                new Twist2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond),
                GRRMath.TWIST2D_0
            )
        ) {
            // - We are stopping.
            stopping = true;

            // - Find the last target's total velocity across the X and Y axis.
            Translation2d lastV = new Translation2d(lastTarget.chassisSpeeds.vxMetersPerSecond, lastTarget.chassisSpeeds.vyMetersPerSecond);

            // - Determine if the last target had a velocity of 0.
            boolean wasMoving = !GRRMath.epsilonEquals(lastV.getNorm(), 0.0);

            // For every module:
            for (int i = 0; i < modulePositions.length; ++i) {
                // - If the robot was moving, set the desired angle to the direction of the
                //   robot's current lateral motion. This is to prevent the robot from veering
                //   if it is stopping from a state of rotating while translating.
                desiredModuleStates[i].angle = wasMoving ? lastV.getAngle() : lastTarget.moduleStates[i].angle;

                // - Translational speed is still 0.
                desiredModuleStates[i].speedMetersPerSecond = 0.0;

                // - Add the heading to overrides.
                overrideHeading.add(Optional.of(desiredModuleStates[i].angle));
            }
        }

        // - Declare arrays to be hydrated with calculated module velocities and headings.
        double[] lastVx = new double[modulePositions.length];
        double[] lastVy = new double[modulePositions.length];
        Rotation2d[] lastHeading = new Rotation2d[modulePositions.length];
        double[] desiredVx = new double[modulePositions.length];
        double[] desiredVy = new double[modulePositions.length];
        Rotation2d[] desiredHeading = new Rotation2d[modulePositions.length];

        // - Declare a tripwire to determine if all modules are flipping with the
        //   desired speeds (indicative of moving in the opposite direction within
        //   90 degrees). If this remains true, it is probably faster to start
        //   over with a velocity of 0.
        boolean allFlip = true;

        // (Special behavior for translation direction changes in the
        // opposite direction of current movement within 90 degrees)
        // For every module:
        for (int i = 0; i < modulePositions.length; ++i) {
            // - Find the last module velocity and heading using the last module states.
            lastVx[i] = lastTarget.moduleStates[i].angle.getCos() * lastTarget.moduleStates[i].speedMetersPerSecond;
            lastVy[i] = lastTarget.moduleStates[i].angle.getSin() * lastTarget.moduleStates[i].speedMetersPerSecond;
            lastHeading[i] = lastTarget.moduleStates[i].angle;

            // If the last module velocity was in reverse:
            //     - Flip the heading.
            if (lastTarget.moduleStates[i].speedMetersPerSecond < 0.0) {
                lastHeading[i] = Rotation2d.fromRadians(MathUtil.angleModulus(lastHeading[i].getRadians() + Math.PI));
            }

            // - Find the desired module velocity and heading using the desired module states.
            desiredVx[i] = desiredModuleStates[i].angle.getCos() * desiredModuleStates[i].speedMetersPerSecond;
            desiredVy[i] = desiredModuleStates[i].angle.getSin() * desiredModuleStates[i].speedMetersPerSecond;
            desiredHeading[i] = desiredModuleStates[i].angle;

            // If the desired module velocity is in reverse:
            //     - Flip the heading.
            if (desiredModuleStates[i].speedMetersPerSecond < 0.0) {
                desiredHeading[i] = Rotation2d.fromRadians(MathUtil.angleModulus(desiredHeading[i].getRadians() + Math.PI));
            }

            // If the desired target doesn't require a flip and the allFlip tripwire has not been hit:
            //     - Set the allFlip tripwire to false.
            if (allFlip && Math.abs(lastHeading[i].times(-1.0).rotateBy(desiredHeading[i]).getRadians()) < GRRMath.HALF_PI) {
                allFlip = false;
            }
        }

        // If the allFlip tripwire has been hit, and the last and desired target contains movement:
        //     - Start over with a desired speed of 0 (this should be faster).
        if (
            allFlip &&
            !GRRMath.twist2dEpsilonEquals(
                new Twist2d(
                    lastTarget.chassisSpeeds.vxMetersPerSecond,
                    lastTarget.chassisSpeeds.vyMetersPerSecond,
                    lastTarget.chassisSpeeds.omegaRadiansPerSecond
                ),
                GRRMath.TWIST2D_0
            ) &&
            !GRRMath.twist2dEpsilonEquals(
                new Twist2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond),
                GRRMath.TWIST2D_0
            )
        ) {
            return calculate(lastTarget, new ChassisSpeeds());
        }

        // - Determine the delta between the desired speeds and the last target's speeds.
        double dx = desiredSpeeds.vxMetersPerSecond - lastTarget.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredSpeeds.vyMetersPerSecond - lastTarget.chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredSpeeds.omegaRadiansPerSecond - lastTarget.chassisSpeeds.omegaRadiansPerSecond;

        // - Declare the "s" value. "s" is an interpolation between the desired speeds and the last target's speeds. It
        //   will be lowered based on kinematic constraints. Applied to the final chassis speeds and module states before
        //   they are overridden.
        double s = 1.0;

        // - Find the maximum feasible delta in the a module's heading.
        //   Acceleration is assumed to be a non-factor.
        double maxMrvDelta = 0.020 * 30.0;

        // (Limit s based on module turning constraints)
        // For every module:
        for (int i = 0; i < modulePositions.length; ++i) {
            // If we are stopping:
            //     - Exit the loop.
            if (stopping) {
                break;
            }

            // - Add an empty Optional to the override turning array (this is to ensure null values are not retrieved).
            overrideHeading.add(Optional.empty());

            // If the last target's velocity is 0:
            if (GRRMath.epsilonEquals(lastTarget.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If the desired target's velocity is 0:
                //     - Add the module to the overrideHeading list with the last target's heading and skip the rest of the loop.
                if (GRRMath.epsilonEquals(desiredModuleStates[i].speedMetersPerSecond, 0.0)) {
                    overrideHeading.set(i, Optional.of(lastTarget.moduleStates[i].angle));
                    continue;
                }

                // - Find the rotational delta between the last and desired module heading.
                Rotation2d rotDelta = lastTarget.moduleStates[i].angle.times(-1.0).rotateBy(desiredModuleStates[i].angle);

                // If the rotational delta is greater than PI / 2 (90 degrees) and should be optimized:
                //     - Rotate the delta to an optimized position.
                if (flipHeading(rotDelta)) rotDelta = rotDelta.rotateBy(GRRMath.ROTATION2D_PI);

                // - Find the number of periodic loops needed to turn the module to the desired target.
                double numStepsNeeded = Math.abs(rotDelta.getRadians()) / maxMrvDelta;

                // If the desired target will a single periodic loop to achieve:
                //     - Add the module to the overrideHeading list with the desired target's
                //       heading and skip the rest of the loop. Calculating a reduction for s
                //       shouldn't result in a meaningful change.
                //     Else:
                //         - Add the module to the overrideHeading list with a
                //           heading achievable in the next periodic loop.
                //         - s is now 0, as we wait for all modules to move before accelerating
                //           from a stop. This prevents the "wiggle" typically seen in swerve
                //           drive after accelerating from a dead stop.
                if (numStepsNeeded <= 1.0) {
                    overrideHeading.set(i, Optional.of(desiredModuleStates[i].angle));
                    continue;
                } else {
                    overrideHeading.set(
                        i,
                        Optional.of(
                            lastTarget.moduleStates[i].angle.rotateBy(
                                    Rotation2d.fromRadians(Math.signum(rotDelta.getRadians()) * maxMrvDelta)
                                )
                        )
                    );
                    s = 0.0;
                    continue;
                }
            }

            // - If s is 0, skip the rest of the loop.
            if (s == 0.0) continue;

            // - Find the module's max heading s (refer to the method below on how this is derived).
            double maxS = getHeadingS(
                lastVx[i],
                lastVy[i],
                lastHeading[i].getRadians(),
                desiredVx[i],
                desiredVy[i],
                desiredHeading[i].getRadians(),
                maxMrvDelta
            );

            // - Set s to the minimum of its current value and the
            //   calculated maximum feasible s value of the module.
            s = Math.min(s, maxS);
        }

        // - Find the max feasible delta in the robot's velocity.
        double maxVDelta = 0.020 * 7.6
        ;

        // (Limit s based on velocity constraints)
        // For every module:
        for (int i = 0; i < modulePositions.length; ++i) {
            // If s is already 0:
            //     - Exit.
            if (s == 0.0) break;

            // - Calculate the desaturated x and y velocity based on the
            //   current s value.
            double vxMinS = s == 1.0 ? desiredVx[i] : (desiredVx[i] - lastVx[i]) * s + lastVx[i];
            double vyMinS = s == 1.0 ? desiredVy[i] : (desiredVy[i] - lastVy[i]) * s + lastVy[i];

            // - Find the module's max velocity s (refer to the method below on how this is derived).
            double maxS = s * getVelocityS(lastVx[i], lastVy[i], vxMinS, vyMinS, maxVDelta);

            // - Set s to the minimum of its current value and the
            //   calculated maximum feasible s value of the module.
            s = Math.min(s, maxS);
        }

        // - Declare the constrained speeds by adding percentage s of the
        //   last and desired delta to the last speeds.
        ChassisSpeeds constrainedSpeeds = new ChassisSpeeds(
            lastTarget.chassisSpeeds.vxMetersPerSecond + (s * dx),
            lastTarget.chassisSpeeds.vyMetersPerSecond + (s * dy),
            lastTarget.chassisSpeeds.omegaRadiansPerSecond + (s * dtheta)
        );

        // - Compute module states from constrained speeds..
        SwerveModuleState[] constrainedStates = kinematics.toSwerveModuleStates(constrainedSpeeds);

        // (Module overrides / "global" heading optimization)
        // For every module:
        for (int i = 0; i < modulePositions.length; ++i) {
            // - Get the module's override.
            Optional<Rotation2d> couldOverride = overrideHeading.get(i);

            // If the module is overridden:
            if (couldOverride.isPresent()) {
                // - Get the override.
                Rotation2d override = couldOverride.get();

                // - If the override is a flipped heading, reverse the module's speed.
                if (flipHeading(constrainedStates[i].angle.times(-1.0).rotateBy(override))) {
                    constrainedStates[i].speedMetersPerSecond *= -1.0;
                }

                // - Set the module to the overridden angle.
                constrainedStates[i].angle = override;
            }

            // - Find the module's change in heading.
            Rotation2d deltaRotation = lastTarget.moduleStates[i].angle.times(-1.0).rotateBy(constrainedStates[i].angle);

            // If the change in heading is greater than 90 degrees:
            if (flipHeading(deltaRotation)) {
                // - Optimize the heading and reverse the module's speed.
                constrainedStates[i].angle =
                    Rotation2d.fromRadians(MathUtil.angleModulus(constrainedStates[i].angle.getRadians() + Math.PI));
                constrainedStates[i].speedMetersPerSecond *= -1.0;
            }
        }

        // - Return the constrained speeds.
        return new GRRSwerveTarget(constrainedSpeeds, constrainedStates);
    }

    /**
     * If a heading should be optimized.
     * @param heading The heading to check.
     */
    private boolean flipHeading(Rotation2d heading) {
        return Math.abs(heading.getRadians()) > GRRMath.HALF_PI;
    }

    /**
     * Calculates the maximum {@code s} value (percent of difference between last and desired target) for heading.
     * @param lastVx Last target {@code x} velocity.
     * @param lastVy Last target {@code y} velocity.
     * @param lastHeading Last target heading.
     * @param desiredVx Desired {@code x} velocity.
     * @param desiredVy Desired {@code y} velocity.
     * @param desiredHeading Desired heading.
     * @param maxRotationalVelocityStep The maximum allowed difference in rotational velocity in a periodic loop iteration.
     * @param maxIterations The maximum number of iterations to run the false position method for root finding.
     */
    private double getHeadingS(
        double lastVx,
        double lastVy,
        double lastHeading,
        double desiredVx,
        double desiredVy,
        double desiredHeading,
        double maxRotationalVelocityStep
    ) {
        // - Make sure the headings are optimized.
        desiredHeading = GRRMath.wrapAbout(lastHeading, desiredHeading);

        // - Find the difference between the last and desired headings.
        double diff = desiredHeading - lastHeading;

        // - If the last and desired heading is achievable in one
        //   periodic loop, return 1 as no interpolation is needed.
        if (Math.abs(diff) <= maxRotationalVelocityStep) return 1.0;

        // - Find an achievable heading.
        double achievableHeading = lastHeading + Math.signum(diff) * maxRotationalVelocityStep;

        // - Describes the following function:
        //     - x is the robot's X velocity.
        //     - y is the robot's Y velocity.
        //     Returns:
        //         - Get the robot's velocity as a heading.
        //         - Localize the robot's velocity heading around the last module heading.
        //         - Subtract the achievable heading.
        GRRMath.Parametric func = (x, y) -> GRRMath.wrapAbout(lastHeading, Math.atan2(y, x)) - achievableHeading;

        // - Solves the above function for its root within the bounds of the last
        //   and desired chassis speeds. Colloquially, this solves for a percentage to
        //   reduce the robot's velocity by to ensure that undesirable behavior does not
        //   arise from modules pulling the robot in unpredictable directions during
        //   the heading transitional period.
        return GRRMath.findRoot(
            func,
            lastVx,
            lastVy,
            lastHeading - achievableHeading,
            desiredVx,
            desiredVy,
            desiredHeading - achievableHeading,
            8
        );
    }

    /**
     * Calculates the maximum {@code s} value (percent of difference between last and desired target) for velocity.
     * @param lastVx Last target {@code x} velocity.
     * @param lastVy Last target {@code y} velocity.
     * @param desiredVx Desired {@code x} velocity.
     * @param desiredVy Desired {@code y} velocity.
     * @param maxVelocityStep The maximum allowed difference in velocity in a periodic loop iteration.
     * @param maxIterations The maximum number of iterations to run the false position method for root finding.
     */
    private double getVelocityS(double lastVx, double lastVy, double desiredVx, double desiredVy, double maxVelocityStep) {
        // - Compute the last and desired translational velocities.
        double lastNorm = Math.hypot(lastVx, lastVy);
        double desiredNorm = Math.hypot(desiredVx, desiredVy);

        // - Find the difference between the last and desired headings.
        double diff = desiredNorm - lastNorm;

        // - If the last and desired velocity is achievable in one
        //   periodic loop, return 1 as no interpolation is needed.
        if (Math.abs(diff) <= maxVelocityStep) return 1.0;

        // - Find an achievable velocity.
        double achievableVelocity = lastNorm + Math.signum(diff) * maxVelocityStep;

        // - Describes the following function:
        //     - x is the robot's X velocity.
        //     - y is the robot's Y velocity.
        //     Returns:
        //         - Find the total velocity.
        //         - Take the difference of the achievable velocity.
        GRRMath.Parametric func = (x, y) -> Math.hypot(x, y) - achievableVelocity;

        // - Solves the above function for its root within the bounds of the
        //   last and desired chassis speeds. This is used to prevent infeasible
        //   acceleration while translating on a per module basis.
        return GRRMath.findRoot(
            func,
            lastVx,
            lastVy,
            lastNorm - achievableVelocity,
            desiredVx,
            desiredVy,
            desiredNorm - achievableVelocity,
            10
        );
    }
}