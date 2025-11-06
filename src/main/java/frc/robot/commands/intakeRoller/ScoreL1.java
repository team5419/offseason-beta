package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;

public class ScoreL1 extends SequentialCommandGroup {
    private final IntakePivot intakePivot;

    public ScoreL1(RobotContainer robot) {
        intakePivot = robot.getIntakePivot();

        addRequirements(intakePivot);

        addCommands(
                new InstantCommand(() -> intakePivot.runPosition(IntakePivotGoal.L1)),
                new WaitCommand(0.7),
                new OuttakeCoral(robot));
    }
}
