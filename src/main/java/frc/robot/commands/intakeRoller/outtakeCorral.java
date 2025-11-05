package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class OuttakeCorral extends Command {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;

    public OuttakeCorral(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();

        addRequirements(roller);
    }

    public void initialize() {
        roller.setGoal(IntakeRollerGoal.OUTTAKECORRAL);
    }

    public void execute() {}

    public boolean isFinished() {
        return !beamBreak.coralInIntake();
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
    }
}
