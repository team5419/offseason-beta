package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivot.IntakePivotGoal;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class RunIntake extends Command {

    private static final double kTimeOutTime = 3;

    private final IntakeRoller roller;
    private final Beambreak beamBreak;
    private final IntakePivot pivot;
    private final Timer timer;

    public RunIntake(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
        pivot = robot.getIntakePivot();
        addRequirements(roller, pivot);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
        roller.setCurrentGoal(IntakeRollerGoal.INTAKE);
        pivot.setCurrentGoal(IntakePivotGoal.INTAKE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return beamBreak.coralInIntake() || timer.hasElapsed(kTimeOutTime);
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(IntakeRollerGoal.IDLE);
    }
}
