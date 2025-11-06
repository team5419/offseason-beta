package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;

public class AlignForHandoff extends Command {

    private final Wrist wrist;
    private final IntakePivot pivot;

    public AlignForHandoff(RobotContainer robot) {
        pivot = robot.getIntakePivot();
        wrist = robot.getWrist();

        addRequirements(pivot, wrist);
    }

    @Override
    public void initialize() {
        pivot.setCurrentGoal(IntakePivotGoal.TO_INTAKE_HANDOFF);
        wrist.setCurrentGoal(WristGoal.HANDOFF);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return pivot.getCurrentGoal() == IntakePivotGoal.TO_INTAKE_HANDOFF
                && wrist.getCurrentGoal() == WristGoal.HANDOFF
                && pivot.atGoal()
                && wrist.atGoal();
    }

    @Override
    public void end(boolean isFinished) {}
}
