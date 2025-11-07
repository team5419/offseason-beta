package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.swerve.AutoAlignToCoral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.endeffector.EndEffector.EndEffectorRollerGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(RobotContainer robot, CommandXboxController driver) {

        Elevator elevator = robot.getElevator();
        Wrist wrist = robot.getWrist();

        addRequirements(elevator, wrist);

        addCommands(
                // new InstantCommand(() -> robot.getEndEffector().setCurrentGoal(EndEffectorRollerGoal.)),
                new ConditionalCommand(
                        new ScoreL1(robot),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new AutoAlignToCoral(robot, driver),
                                        new Elevate(robot, () -> elevator.getDesiredLevel())),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.3), new InstantCommand()), // replace w outtake coral
                                        new InstantCommand(),
                                        (() -> RobotState.getInstance().isAutoAlignAtGoal())),
                                new InstantCommand(
                                        () -> RobotState.getInstance().setAutoAlignAtGoal(false))),
                        () -> elevator.getDesiredLevel() == ElevatorGoal.L1),
                new InstantCommand(() -> robot.getEndEffector().setCurrentGoal(EndEffectorRollerGoal.IDLE)));
    }
}
