package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class ScoreCoralL1 extends SequentialCommandGroup {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot intakePivot;

    public ScoreCoralL1(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        intakePivot = robot.getIntakePivot();

        addCommands(
                new InstantCommand(() -> intakePivot.runPosition(IntakePivotGoal.TO_SCOREL1)),
                new WaitCommand(0.7),
                new InstantCommand(() -> roller.setGoal(IntakeRollerGoal.OUTTAKECORRAL)),
                new WaitUntilCommand(() -> !beamBreak.coralInIntake()),
                new InstantCommand(() -> roller.setGoal(IntakeRollerGoal.IDLE)) // I dont know why theirs errors
                );
    }
}
