package frc.robot.commands.intakeRoller;

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

    public void initialize() {
        endEffector.setGoal(EndEffectorRollerGoal.INTAKE);
    }

    public boolean isFinished() {
        return beambreak.gamepieceInEndEffector();
    }

    public void end() {
        endEffector.setGoal(EndEffectorRollerGoal.HOLD);
    }
}
