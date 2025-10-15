package frc.robot.commands.Intake.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import java.util.function.Supplier;

public class TakePivotToHandoff extends Command {

    private final IntakePivot intake;
    public final Supplier<IntakePivotGoal> goal;

    public TakePivotToHandoff(RobotContainer robot, Supplier<IntakePivotGoal> goal) {
        intake = robot.getIntakePivot();

        this.goal = goal;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setCurrentGoal(IntakePivotGoal.INTAKE);
    }

    @Override
    public void execute() {
        intake.setCurrentGoal(goal.get());
    }

    @Override
    public boolean isFinished() {
        return intake.atGoal();
    }

    @Override
    public void end(boolean isFinished) {
        intake.setCurrentGoal(IntakePivotGoal.IDLE);
    }
}
