package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RaiseToPos;
import frc.robot.commands.intake.intakeroller.OuttakeCoral;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import java.util.function.Supplier;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(RobotContainer robot, Supplier<ElevatorGoal> elevatorGoal, Supplier<WristGoal> wristGoal) {
        addRequirements(robot.getElevator(), robot.getWrist(), robot.getIntakePivot(), robot.getIntakeRoller());
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(), // auto align
                        new RaiseToPos(robot, elevatorGoal, wristGoal)),
                new OuttakeCoral(robot));
    }
}
