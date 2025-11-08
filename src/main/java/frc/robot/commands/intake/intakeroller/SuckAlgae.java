package frc.robot.commands.intake.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;

public class SuckAlgae extends Command {

    private final EndEffector endEffector;
    private final Beambreak beambreak;

    public SuckAlgae(RobotContainer robot) {
        endEffector = robot.getEndEffector();
        beambreak = robot.getBeamBreak();
    }

    @Override
    public void initialize() {
        endEffector.setCurrentGoal(EndEffectorRollerGoal.INTAKE_ALGAE);
    }

    @Override
    public boolean isFinished() {
        return beambreak.gamepieceInEndEffector();
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setCurrentGoal(EndEffectorRollerGoal.GENTLE_INTAKE);
    }
}
