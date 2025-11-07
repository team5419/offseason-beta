package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import java.util.function.Supplier;

public class RunEndEffector extends Command {

    private final EndEffector endEffector;
    private final Supplier<EndEffectorRollerGoal> goal;

    public RunEndEffector(RobotContainer robot, Supplier<EndEffectorRollerGoal> goal) {
        this.goal = goal;
        this.endEffector = robot.getEndEffector();

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        endEffector.setCurrentGoal(goal.get());
    }

    @Override
    public boolean isFinished() {
        return endEffector.atGoal() && endEffector.getCurrentGoal() == goal.get();
    }

    @Override
    public void end(boolean interrupted) {}
}
