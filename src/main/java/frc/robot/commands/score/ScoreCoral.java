package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(RobotContainer robot) {
        addRequirements(robot.getElevator(), robot.getWrist(), robot.getIntakePivot(), robot.getIntakeRoller());
        addCommands(new ParallelCommandGroup(
                // auto align to reef
                // new RaiseToPosition(robot, null)
                ));
    }
}
