package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.beambreak.Beambreak;

public class IntakeCoral extends SequentialCommandGroup {

    private final Beambreak beamBreak;

    public IntakeCoral(RobotContainer robot) {
        beamBreak = robot.getBeamBreak();
        addCommands(
                new RunIntake(robot), new WaitUntilCommand(() -> beamBreak.coralInIntake()).withTimeout(3), new HandoffCoral(robot));
    }
}
