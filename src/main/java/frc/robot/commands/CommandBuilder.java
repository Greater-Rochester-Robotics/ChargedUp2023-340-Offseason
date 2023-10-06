package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command builder. Very similar to {@link FunctionalCommand}.
 */
public class CommandBuilder extends CommandBase {

    private Runnable onInitialize = () -> {};
    private Runnable onExecute = () -> {};
    private Consumer<Boolean> onEnd = interrupted -> {};
    private Supplier<Boolean> isFinished = () -> false;

    /**
     * Create the command builder.
     * @param requirements The subsystems required by the command.
     */
    public CommandBuilder(Subsystem... requirements) {
        super.addRequirements(requirements);
    }

    public CommandBuilder addSubsystemRequirements(Subsystem... requirements){
        super.addRequirements(requirements);
        return this;
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    public CommandBuilder onInitialize(Runnable onInitialize) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.onInitialize = onInitialize;
        return this;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    public CommandBuilder onExecute(Runnable onExecute) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.onExecute = onExecute;
        return this;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally,
     * or when it interrupted/canceled. Supplied boolean is if the command was interrupted.
     */
    public CommandBuilder onEnd(Consumer<Boolean> onEnd) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.onEnd = onEnd;
        return this;
    }

    /**
     * 
     * @param onEnd
     * @return
     */
    public CommandBuilder onEnd(Runnable onEnd) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.onEnd = (interrupted)->onEnd.run();
        return this;
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it. By default, this returns {@code false}.
     */
    public CommandBuilder isFinished(Supplier<Boolean> isFinished) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.isFinished = isFinished;
        return this;
    }

    public CommandBuilder isFinished(boolean isFinished) {
        if (this.isScheduled()) throw new IllegalStateException("Cannot change methods of a command while it is scheduled");
        this.isFinished = ()->isFinished;
        return this;
    }

    @Override
    public void initialize() {
        onInitialize.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished.get();
    }
}