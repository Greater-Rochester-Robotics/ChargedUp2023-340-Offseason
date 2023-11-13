package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.commands.Routines.*;

import org.team340.lib.GRRDashboard;
import org.team340.lib.control.AdvancedController;
import org.team340.lib.util.RevUtil;
import org.team340.robot.Constants.ArmConstants.Positions;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.SystemsCheck;
import org.team340.robot.subsystems.Arm;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.ScoreTarget;
import org.team340.robot.subsystems.ScoreTarget.Level;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
public final class RobotContainer {

    private RobotContainer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static AdvancedController driver;
    private static AdvancedController coDriver;

    public static Swerve swerve;
    public static Arm arm;
    public static Intake intake;
    public static ScoreTarget scoreTarget;

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
        coDriver =
            new AdvancedController(
                ControllerConstants.CO_DRIVER,
                ControllerConstants.JOYSTICK_DEADBAND,
                ControllerConstants.JOYSTICK_THRESHOLD,
                ControllerConstants.TRIGGER_DEADBAND,
                ControllerConstants.TRIGGER_THRESHOLD
            );

        // Add controllers to the dashboard.
        GRRDashboard.addController("Driver", driver);
        GRRDashboard.addController("CoDriver", coDriver);

        // Initialize subsystems.
        swerve = new Swerve();
        arm = new Arm();
        intake = new Intake();
        scoreTarget = new ScoreTarget();

        // Add subsystems to the dashboard.
        swerve.addToDashboard();
        arm.addToDashboard();
        intake.addToDashboard();
        scoreTarget.addToDashboard();

        // Set systems check command.
        GRRDashboard.setSystemsCheck(SystemsCheck.command());

        // Print successful REV hardware initialization.
        RevUtil.printSuccess();

        // Configure bindings and autos.
        configureBindings();
        configureAutos();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configureBindings() {
        // Set default commands.
        arm.setDefaultCommand(arm.holdPosition());
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        /**
         * Driver bindings.
         */

        driver.a().onTrue(intake(true)).onFalse(storeCube());
        driver.b().onTrue(scoreTarget.getDriverCommand()).onFalse(intake.stopMotors());
        driver.x().onTrue(intake(false)).onFalse(storeCube());
        driver
            .y()
            .onTrue(sequence(scoreTarget.setLevel(Level.FAR), scoreTarget.getCoDriverCommand(), scoreTarget.getDriverCommand()))
            .onFalse(sequence(intake.stopMotors(), arm.setPosition(Positions.SAFE)));

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zero(0.0));

        // Left Bumper => Snap 180
        driver.leftBumper().whileTrue(swerve.driveSnap180(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Right Bumper => Lock wheels
        driver.rightBumper().whileTrue(swerve.lock());

        /**
         * Co-driver bindings.
         */

        coDriver
            .a()
            .onTrue(sequence(scoreTarget.setLevel(Level.LOW), scoreTarget.getCoDriverCommand()))
            .onFalse(arm.setPosition(Positions.SAFE));
        coDriver
            .b()
            .onTrue(sequence(scoreTarget.setLevel(Level.MID), scoreTarget.getCoDriverCommand()))
            .onFalse(arm.setPosition(Positions.SAFE));
        coDriver
            .x()
            .onTrue(sequence(scoreTarget.setLevel(Level.HIGH), scoreTarget.getCoDriverCommand()))
            .onFalse(arm.setPosition(Positions.SAFE));
        coDriver
            .y()
            .onTrue(sequence(scoreTarget.setLevel(Level.FAR), scoreTarget.getCoDriverCommand()))
            .onFalse(arm.setPosition(Positions.SAFE));

        coDriver.leftBumper().whileTrue(arm.setDutyCycle(() -> getArmManualSpeed(), () -> 0.0));
        coDriver.rightBumper().whileTrue(arm.setDutyCycle(() -> 0.0, () -> getWristManualSpeed()));
        coDriver
            .leftBumper()
            .and(coDriver.rightBumper())
            .whileTrue(arm.setDutyCycle(() -> getArmManualSpeed(), () -> getWristManualSpeed()));
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configureAutos() {
        GRRDashboard.addAutoCommand("Example", Autos.example());
        // If an auto uses a PathPlanner path file, make sure to include it.
        // GRRDashboard.addAutoCommand("Example", Autos.example(), "pathFileName");
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
        return driver.getTriggerDifference(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }

    /**
     * Gets the arm's manual speed from the co-driver's controller (left stick Y).
     * @return The speed from -1.0 to 1.0, scaled to the max output.
     */
    public static double getArmManualSpeed() {
        return coDriver.getLeftY(Constants.ArmConstants.ARM_MAX_MANUAL_DUTY_CYCLE);
    }

    /**
     * Gets the wrist's manual speed from the co-driver's controller (right stick Y).
     * @return The speed from -1.0 to 1.0, scaled to the max output.
     */
    public static double getWristManualSpeed() {
        return coDriver.getRightY(Constants.ArmConstants.WRIST_MAX_MANUAL_DUTY_CYCLE);
    }
}
