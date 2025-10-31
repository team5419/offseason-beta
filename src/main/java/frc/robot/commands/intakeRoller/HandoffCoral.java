package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;

public class HandoffCoral extends Command {
    private final IntakeRoller roller;
    private final EndEffector endEffector;
    private final Wrist wrist;
    private final IntakePivot pivot;
    private final Beambreak beamBreak;
    private final Timer timer; 
    private final Double timeOutTime = 10.0;

    public HandoffCoral(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        endEffector = robot.getEndEffector();
        beamBreak = robot.getBeamBreak();
        pivot = robot.getIntakePivot();
        wrist = robot.getWrist();
        timer = new Timer();
    }

    public void initialize() {
        pivot.setGoal(IntakePivotGoal.TO_INTAKE_HANDOFF);
        wrist.setGoal(WristGoal.HANDOFF);
        timer.start();
    }

    public void execute() {
        if (pivot.atGoal() && wrist.atGoal()) {
            roller.setGoal(IntakeRollerGoal.OUTTAKEENDEFFECTOR);
            endEffector.setGoal(EndEffectorRollerGoal.INTAKE);
        }
    }

    public boolean isFinished() {
        return beamBreak.gamepieceInEndEffector() || timer.hasElapsed(timeOutTime); //in case of mechanical failure
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
        // endEffector.setGoal(EndEffectorRollerGoal.IDLE);
    }
}
