package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import java.util.function.Supplier;

public class RaiseToPosition extends Command {
    private final Elevator elevator;
    private final Wrist wrist;
    private final Supplier<ElevatorGoal> positionSupplier;
    private final Supplier<WristGoal> wristSupplier;

    public RaiseToPosition(RobotContainer robot, Supplier<ElevatorGoal> elevatorGoal, Supplier<WristGoal> wristGoal) {
        this.elevator = robot.getElevator();
        this.wrist = robot.getWrist();
        positionSupplier = elevatorGoal;
        wristSupplier = wristGoal;
        addRequirements(elevator, robot.getWrist());
    }

    @Override
    public void execute() {
        elevator.setCurrentGoal(positionSupplier.get());
        wrist.runPosition(wristSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentGoal() == positionSupplier.get() && elevator.atGoal();
    }
}
