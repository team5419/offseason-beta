package frc.robot.commands.intake.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class OuttakeCoral extends Command {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;

    public OuttakeCoral(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();

        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setCurrentGoal(IntakeRollerGoal.OUTTAKECORRAL);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !beamBreak.coralInIntake();
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(IntakeRollerGoal.IDLE);
    }
}
