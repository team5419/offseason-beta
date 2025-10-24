package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.roller.IntakeRoller;

public class intake extends SequentialCommandGroup {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot intakePivot;

    public intake(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        intakePivot = robot.getIntakePivot();

        addCommands(
                // new InstantCommand(() -> intakePivot.setGoal(IntakePivotGoal.TO_INTAKE()))
                new WaitCommand(0.7), new intakeCorral(robot), new WaitCommand(0.7), new intakeToEndEffector(robot));
    }
}
