package frc.robot.commands.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRoller.IntakeRollerGoal;

public class intakeCorral extends Command {
    private final IntakeRoller roller;

    public intakeCorral(RobotContainer robot) {
        roller = robot.getRoller(); // fix later
    }

    public void initialize() {
        roller.setGoal(IntakeRollerGoal.INTAKE);
    }

    public void execute() {}

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {
        roller.setGoal(IntakeRollerGoal.IDLE);
    }
}
