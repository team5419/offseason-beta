package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class OuttakeL1 extends Command {

    private final Beambreak beambreak;
    private final IntakeRoller roller;

    public OuttakeL1(RobotContainer robot) {
        beambreak = robot.getBeamBreak();
        roller = robot.getIntakeRoller();

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
        return !beambreak.coralInIntake();
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(IntakeRollerGoal.IDLE);
    }
}
