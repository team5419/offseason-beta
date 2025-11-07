package frc.robot.commands.outtake.endeffector;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import java.util.function.Supplier;

public class OuttakeAlgae extends Command {

    private final Wrist wrist;
    private final EndEffector endEffector;
    private final Supplier<WristGoal> wristGoal;
    private final Supplier<EndEffectorRollerGoal> endEffectorRollerGoal;

    public OuttakeAlgae(
            RobotContainer robot,
            Supplier<WristGoal> wristGoal,
            Supplier<EndEffectorRollerGoal> endEffectorRollerGoal) {
        this.wristGoal = wristGoal;
        this.endEffectorRollerGoal = endEffectorRollerGoal;
        wrist = robot.getWrist();
        endEffector = robot.getEndEffector();
        addRequirements(wrist, endEffector);
    }

    @Override
    public void execute() {
        wrist.setCurrentGoal(wristGoal.get());
        endEffector.setCurrentGoal(endEffectorRollerGoal.get());
    }

    @Override
    public boolean isFinished() {
        return (endEffector.atGoal() && endEffector.getCurrentGoal() == endEffectorRollerGoal.get() && wrist.atGoal() && wrist.getCurrentGoal() == wristGoal.get());
    }

    @Override
    public void end(boolean isFinished) {}
}
