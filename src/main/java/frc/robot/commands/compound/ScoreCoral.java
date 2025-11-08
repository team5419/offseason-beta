package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RaiseToPos;
import frc.robot.commands.outtake.endeffector.OuttakeCoral;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(RobotContainer robot) {
        addRequirements(robot.getElevator(), robot.getWrist(), robot.getEndEffector());
        addCommands(new RaiseToPos(robot, () -> robot.getElevator().getDesiredLevel()), new OuttakeCoral(robot));
    }
}
