package frc.robot.commands.score;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RaiseToPosition;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(RobotContainer robot, Supplier<ElevatorGoal> elevatorGoal, Supplier<WristGoal> wristGoal) {
        addRequirements(robot.getElevator(), robot.getWrist(), robot.getIntakePivot(), robot.getIntakeRoller());
        addCommands(new ParallelCommandGroup(
                // TODO: auto align to reef
                new RaiseToPosition(robot, elevatorGoal, wristGoal)
                // TODO: outtake coral
        ));
    }
}
