package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;

public class HandoffCoral extends Command {

    private static final double kTimeOutTime = 3.0;
    private final IntakeRoller roller;
    private final EndEffector endEffector;
    private final Beambreak beamBreak;
    private final Timer timer;

    public HandoffCoral(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        endEffector = robot.getEndEffector();
        beamBreak = robot.getBeamBreak();
        timer = new Timer();

        addRequirements(roller, endEffector);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        roller.setCurrentGoal(IntakeRollerGoal.OUTTAKEENDEFFECTOR);
        endEffector.setCurrentGoal(EndEffectorRollerGoal.HANDOFF);
    }

    @Override
    public boolean isFinished() {
        return (beamBreak.gamepieceInEndEffector() && !beamBreak.coralInIntake()) || timer.hasElapsed(kTimeOutTime);
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(IntakeRollerGoal.IDLE);
    }
}
