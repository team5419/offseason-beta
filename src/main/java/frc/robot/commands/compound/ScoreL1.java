package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.OuttakeL1;
import frc.robot.commands.Intake.intakepivot.IntakePivotToPosition;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;

public class ScoreL1 extends SequentialCommandGroup {
    private final IntakePivot intakePivot;

    public ScoreL1(RobotContainer robot) {
        intakePivot = robot.getIntakePivot();

        addRequirements(intakePivot);

        addCommands(
                new IntakePivotToPosition(robot, () -> IntakePivotGoal.SCORE_L1),
                new OuttakeL1(robot),
                new IntakePivotToPosition(robot, () -> IntakePivotGoal.HANDOFF));
    }
}
