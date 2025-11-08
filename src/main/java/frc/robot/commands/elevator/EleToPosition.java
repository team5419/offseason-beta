package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import java.util.function.Supplier;

public class EleToPosition extends Command {
    private final Elevator elevator;
    private final Supplier<ElevatorGoal> positionSupplier;

    public EleToPosition(Elevator elevator, Supplier<ElevatorGoal> goal) {
        this.elevator = elevator;
        positionSupplier = goal;
        addRequirements(elevator);
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
