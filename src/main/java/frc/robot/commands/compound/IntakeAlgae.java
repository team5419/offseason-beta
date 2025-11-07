package frc.robot.commands.compound;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.intakeroller.SuckAlgae;
import frc.robot.commands.swerve.DriveToPos;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import frc.robot.subsystems.swerve.Swerve;

public class IntakeAlgae extends SequentialCommandGroup {

    private static double kOffsetIntoReef = -0.10;
    
    private Elevator elevator;
    private Swerve swerve;

    public IntakeAlgae(RobotContainer robot, CommandXboxController driver) {
        elevator = robot.getElevator();
        swerve = robot.getSwerve();

        addRequirements();

        addCommands(
            new ParallelCommandGroup(new DriveToPos(
                                robot,
                                driver,
                                () -> swerve.getBestReefTagNoOffset()
                                        .plus(GeomUtil.toTransform2d(new Translation2d(0.85, kOffsetIntoReef)))
                                        .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))),
                                () -> false,
                                () -> false), new Elevate(robot, () -> WristGoal.ALGAE, () -> elevator.getDesiredLevel())),
            new SuckAlgae(robot)
        );
    }
}
