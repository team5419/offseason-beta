import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RaiseToPosition;
import frc.robot.commands.outtake.endeffector.OuttakeAlgae;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;

public class ScoreAlgae extends SequentialCommandGroup {
    public ScoreAlgae(RobotContainer robot) {
        addCommands(
            new RaiseToPosition(robot, () -> ElevatorGoal.L4, () -> WristGoal.HANDOFF),
            new OuttakeAlgae(robot, () -> WristGoal.ALGAE, () -> EndEffectorRollerGoal.OUTTAKE)
        );
    }
}
