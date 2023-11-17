package org.team340.robot;

import static org.team340.robot.commands.Routines.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.GRRDashboard;
import org.team340.lib.control.AdvancedController;
import org.team340.lib.util.RevUtil;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.subsystems.Arm;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
public final class RobotContainer {

    private RobotContainer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static AdvancedController driver;

    public static Swerve swerve;
    public static Arm arm;
    public static Intake intake;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public static void init() {
        // Initialize controllers.
        driver =
            new AdvancedController(
                ControllerConstants.DRIVER,
                ControllerConstants.JOYSTICK_DEADBAND,
                ControllerConstants.JOYSTICK_THRESHOLD,
                ControllerConstants.TRIGGER_DEADBAND,
                ControllerConstants.TRIGGER_THRESHOLD
            );

        // Add controllers to the dashboard.
        GRRDashboard.addController("Driver", driver);

        // Initialize subsystems.
        swerve = new Swerve();
        arm = new Arm();
        intake = new Intake();

        // Add subsystems to the dashboard.
        swerve.addToDashboard();
        arm.addToDashboard();
        intake.addToDashboard();

        // Print successful REV hardware initialization.
        RevUtil.printSuccess();

        // Configure bindings.
        configureBindings();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configureBindings() {
        // Set default commands.
        arm.setDefaultCommand(arm.holdPosition());
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        // Triggers.
        new Trigger(DriverStation::isDisabled).onTrue(arm.setBrakeMode(true)).onFalse(arm.setBrakeMode(false));

        /**
         * Driver bindings.
         */

        // A => Intake
        driver.a().onTrue(intake()).onFalse(storeCube());

        // B => Shoot slow
        driver.b().onTrue(shootSlow()).onFalse(stopShooting());

        // X => Shoot normal
        driver.x().onTrue(shootNormal()).onFalse(stopShooting());

        // Y => Shoot fast
        driver.y().onTrue(shootFast()).onFalse(stopShooting());

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zero(0.0));

        // Left Bumper => Snap 180
        driver.leftBumper().whileTrue(swerve.driveSnap180(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Right Bumper => Lock wheels
        driver.rightBumper().whileTrue(swerve.lock());
    }

    /**
     * Gets the X axis drive speed from the driver's controller.
     */
    private static double getDriveX() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftY(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the Y axis drive speed from the driver's controller.
     */
    private static double getDriveY() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftX(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the rotational drive speed from the driver's controller.
     */
    private static double getDriveRotate() {
        double multiplier =
            (
                (driver.getHID().getLeftStickButton())
                    ? ControllerConstants.DRIVE_ROT_MULTIPLIER_MODIFIED
                    : ControllerConstants.DRIVE_ROT_MULTIPLIER
            );
        return driver.getTriggerDifference(multiplier, ControllerConstants.DRIVE_ROT_EXP);
    }
}
