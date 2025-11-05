package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.Wrist.WristGoal;
import java.util.function.Supplier;

public class RaiseToPos extends Command {

    private final Wrist wrist;
    private final Elevator elevator;
    private final Supplier<WristGoal> wristGoal;
    private final Supplier<ElevatorGoal> eleGoal;

    public RaiseToPos(RobotContainer robot, Supplier<WristGoal> wristGoal, Supplier<ElevatorGoal> eleGoal) {
        this.wristGoal = wristGoal;
        this.eleGoal = eleGoal;
        wrist = robot.getWrist();
        elevator = robot.getElevator();
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.runPosition(wristGoal.get());
        elevator.runPosition(eleGoal.get());
    }

    @Override
    public boolean isFinished() {
        return wrist.atGoal();
        return elevator.atGoal();
    }

    @Override
    public void end(boolean isFinished) {
        wrist.setCurrentGoal(WristGoal.IDLE);
        elevator.setCurrentGoal(ElevatorGoal.IDLE);
    }
}
