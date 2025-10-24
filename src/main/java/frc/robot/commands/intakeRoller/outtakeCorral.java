package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class outtakeCorral extends Command {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;

    public outtakeCorral(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeamBreak();
    }

    public void initialize() {
        roller.setGoal(IntakeRollerGoal.OUTTAKECORRAL);
    }

    public void execute() {}

    public boolean isFinished() {
        if (beamBreak.coralInIntake() == false) {
            return true;
        } else {
            return false;
        }
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
    }
}
