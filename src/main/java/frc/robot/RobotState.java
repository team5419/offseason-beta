package frc.robot;

import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    @Setter
    @Getter
    @AutoLogOutput(key = "RobotState/Is vision connected")
    private boolean visionConnected;

    @Getter
    @Setter
    private boolean isAutoFinished;

    @Setter
    @Getter
    private boolean hasCoral;

    @Setter
    @Getter
    @AutoLogOutput(key = "RobotState/Is aligning")
    private boolean aligning;

    @Getter
    @Setter
    private boolean autoAlignAtGoal;

    @Setter
    @Getter
    @AutoLogOutput(key = "RobotState/Elevator Level")
    private ElevatorGoal elevatorLevel;

    @Setter
    @Getter
    @AutoLogOutput(key = "RobotState/Is early")
    private boolean isEarly;
}
