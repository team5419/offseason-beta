package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import frc.robot.subsystems.beambreak.Beambreak;

public class intakeToEndEffector extends Command {
    private final IntakeRoller roller;
    private final EndEffector endEffector;
    private final Beambreak beamBreak;

    public intakeToEndEffector(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        endEffector = robot.getEndEffector();
        beamBreak = robot.getBeamBreak();
    }

    public void initialize() {
        roller.setGoal(IntakeRollerGoal.OUTTAKEENDEFFECTOR);
        //endEffector.setGoal(EndEffectorRollerGoal.INTAKE);
    }

    public void execute() {}

    public boolean isFinished() {
        return beamBreak.gamepieceInEndEffector();
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
        //endEffector.setGoal(EndEffectorRollerGoal.IDLE);
    }
}
