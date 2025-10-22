package frc.robot.commands.intakeRoller;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.commands.intakeRoller.intakeCorral;

public class intake extends SequentialCommandGroup {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot intakePivot; 

    public scoreCoralL1(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        intakePivot = robot.getIntakePivot();

        addCommands(
            //new InstantCommand(() -> intakePivot.setGoal(IntakePivotGoal.TO_INTAKE()))
            new WaitCommand(0.7),
            new intakeCorral(robot),
            new WaitCommand(0.7),
            //new InstantCommand(() -> intakePivot.setGoal(IntakePivotGoal.TO_HANDOFF()))
            new WaitCommand(0.7)


        );
    }

