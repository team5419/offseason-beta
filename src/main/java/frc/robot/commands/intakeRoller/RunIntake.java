package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class RunIntake extends Command {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot pivot;

    public RunIntake(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        pivot = robot.getIntakePivot();
    }

    public void initialize() {
        roller.setGoal(IntakeRollerGoal.INTAKE);
        pivot.setGoal(IntakePivotGoal.TO_INTAKE);
    }

    public void execute() {}

    public boolean isFinished() {
        return beamBreak.coralInIntake();
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
        pivot.setGoal(IntakePivotGoal.IDLE);
    }
}
