package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.AlignForHandoff;
import frc.robot.commands.intake.HandoffCoral;
import frc.robot.commands.intake.RunIntake;

public class IntakeCoral extends SequentialCommandGroup {

    public IntakeCoral(RobotContainer robot) {
        addCommands(new RunIntake(robot), new AlignForHandoff(robot), new HandoffCoral(robot));
    }
}
