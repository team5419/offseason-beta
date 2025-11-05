package frc.robot.commands.swerve.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import java.util.function.Supplier;

public class RunAtVelocity extends Command {

    private final EndEffector endEffector;

    private final Supplier<EndEffectorRollerGoal> goal;

    public RunAtVelocity(EndEffector endEffector, Supplier<EndEffectorRollerGoal> goal) {
        this.goal = goal;
        this.endEffector = endEffector;

        addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return endEffector.atGoal();
    }

    @Override
    public void end(boolean interrupted) {}
}
