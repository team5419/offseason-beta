package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.roller.IntakeRoller;

public class Intake extends SequentialCommandGroup {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot intakePivot;

    public Intake(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        intakePivot = robot.getIntakePivot();
        addRequirements(roller,beamBreak,intakePivot);

        addCommands(
                // new InstantCommand(() -> intakePivot.setGoal(IntakePivotGoal.TO_INTAKE()))
                new WaitCommand(0.7), new IntakeCorral(robot), new WaitCommand(0.7), new IntakeToEndEffector(robot));
    }
}
