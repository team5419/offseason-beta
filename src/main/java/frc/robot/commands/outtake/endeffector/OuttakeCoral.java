package frc.robot.commands.outtake.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;

public class OuttakeCoral extends Command {
    private final EndEffector roller;
    private final Beambreak beamBreak;

    public OuttakeCoral(RobotContainer robot) {
        roller = robot.getEndEffector();
        beamBreak = robot.getBeamBreak();

        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setCurrentGoal(EndEffectorRollerGoal.OUTTAKE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !beamBreak.gamepieceInEndEffector();
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(EndEffectorRollerGoal.IDLE);
    }
}
