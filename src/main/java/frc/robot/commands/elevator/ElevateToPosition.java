package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;

public class ElevateToPosition extends Command {
    private final Elevator elevator;
    private final Supplier<ElevatorGoal> positionSupplier;

    public ElevateToPosition(Elevator elevator, Supplier<ElevatorGoal> goal) {
        this.elevator = elevator;
        positionSupplier = goal;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setCurrentGoal(positionSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentGoal() == positionSupplier.get() && elevator.atGoal();
    }
}
