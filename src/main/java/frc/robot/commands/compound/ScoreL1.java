package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.OuttakeL1;
import frc.robot.commands.intake.intakepivot.IntakePivotToPosition;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;

public class ScoreL1 extends SequentialCommandGroup {
    private final IntakePivot intakePivot;

    public ScoreL1(RobotContainer robot) {
        intakePivot = robot.getIntakePivot();

        addRequirements(intakePivot);

        addCommands(
                new IntakePivotToPosition(robot, () -> IntakePivotGoal.TO_SCOREL1),
                new OuttakeL1(robot),
                new InstantCommand(() -> robot.getIntakePivot().setCurrentGoal(IntakePivotGoal.TO_INTAKE_HANDOFF)));
    }
}
