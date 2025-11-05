package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import java.util.function.Supplier;
import frc.robot.RobotContainer;

public class RaiseToPosition extends Command {
    private final Elevator elevator;
    private final Supplier<ElevatorGoal> positionSupplier;

    public RaiseToPosition(RobotContainer robot, Supplier<ElevatorGoal> goal) {
        this.elevator = robot.getElevator();
        positionSupplier = goal;
        addRequirements(elevator, robot.getWrist());
    }

    @Override
    public void execute() {
        elevator.setCurrentGoal(positionSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentGoal() == positionSupplier.get() && elevator.atGoal();
    }
}
