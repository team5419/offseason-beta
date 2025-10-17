package frc.robot.commands.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class scoreCoralL1 extends Command {
    private final IntakeRoller roller;
    private final Beambreak beamBreak;

    public scoreCoralL1(RobotContainer robot) {
        roller = robot.getIntakeRoller(); // fix later
        beamBreak = robot.getBeambreak();
    }

    public void initialize() {
        
    }

    public void execute() {}

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
    }
}