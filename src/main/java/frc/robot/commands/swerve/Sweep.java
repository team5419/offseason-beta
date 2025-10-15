package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Sweep extends SequentialCommandGroup{
    public Sweep(RobotContainer robot){
        PathPlannerPath sweepPath = PathPlannerPath.fromPathFile("sweepFile");
        Pose2d reefPose = new Pose2d();
        Command sweepCommand = AutoBuilder.followPath(sweepPath);
        addCommands(
            new DoUntilCommand(
                new DriveToPose(robot, robot.getDriver(), () -> reefPose, () -> false),
                sweepCommand,
                () -> beambreak.broken()
            ));
    }
}
