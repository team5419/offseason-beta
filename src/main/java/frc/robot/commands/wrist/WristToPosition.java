package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import java.util.function.Supplier;

public class WristToPosition extends Command {

    private final Wrist wrist;
    private final Supplier<WristGoal> goal;

    public WristToPosition(RobotContainer robot, Supplier<WristGoal> goal) {
        this.goal = goal;
        wrist = robot.getWrist();
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setCurrentGoal(goal.get());
    }

    @Override
    public boolean isFinished() {
        return wrist.atGoal();
    }

    @Override
    public void end(boolean isFinished) {}
}
