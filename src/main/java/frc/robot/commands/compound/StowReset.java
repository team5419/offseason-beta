package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RaiseToPos;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;

public class StowReset extends SequentialCommandGroup {

    private static final double kStowHeight = 0; // TODO: tune
    private static final double kStowTolerance = kStowHeight + 0.1; // TODO: tune
    private static final double kStowVolts = -0.8; // TODO: tune
    public static final double kIsStopped = 0.002; // rotations per sec // TODO: tune

    private static Elevator elevator;

    public StowReset(RobotContainer robot) {

        elevator = robot.getElevator();
        addRequirements(elevator);

        addCommands(
                new RaiseToPos(robot, () -> ElevatorGoal.STOW),
                new WaitUntilCommand(() -> elevator.getPosition() < kStowTolerance),
                new ParallelCommandGroup(new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.runVolts(kStowVolts)),
                        new WaitCommand(0.02),
                        new WaitUntilCommand(() -> Math.abs(elevator.getAvgVelocity()) <= kIsStopped),
                        new InstantCommand(() -> elevator.stop()),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> elevator.resetPosition()))));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
