package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

public class SimVisualizer {

    private static final double MECHANISM_WIDTH = Units.feetToMeters(6);
    private static final double MECHANISM_HEIGHT = Units.feetToMeters(10);
    private static final Color8Bit MECHANISM_COLOR = new Color8Bit(Color.kBlack);

    private static final double ORIGIN_X = Units.inchesToMeters(12);
    private static final double ORIGIN_Y = Units.inchesToMeters(2.394);

    private static final double intakePivotOffsetX = Units.inchesToMeters(22.746);
    private static final double intakePivotOffsetY = Units.inchesToMeters(9.577);

    private static final double elevatorToGround = Units.inchesToMeters(5.36);
    private static final double startingElevatorHeight = 0.9;
    private static final double mechanismOffset = Units.inchesToMeters(1);

    private final LoggedMechanism2d mechanism2d;

    // Roots
    private final LoggedMechanismRoot2d elevatorRoot;
    private final LoggedMechanismRoot2d intakeRoot;

    // Ligaments
    private final LoggedMechanismLigament2d elevatorMeasured;
    private final LoggedMechanismLigament2d wristMeasured;
    private final LoggedMechanismLigament2d intakeMeasured;

    private final RobotContainer robot;

    public SimVisualizer(RobotContainer robot) {
        this.robot = robot;

        mechanism2d = new LoggedMechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT, MECHANISM_COLOR);

        elevatorRoot = mechanism2d.getRoot("Elevator Root", ORIGIN_X, ORIGIN_Y + mechanismOffset);

        intakeRoot = mechanism2d.getRoot("Intake Root", ORIGIN_X + intakePivotOffsetX, ORIGIN_Y + intakePivotOffsetY);

        elevatorMeasured = elevatorRoot.append(new LoggedMechanismLigament2d(
                "Elevator Measured", startingElevatorHeight, 90, 6.0, new Color8Bit(Color.kFirstBlue)));

        wristMeasured = elevatorMeasured.append(new LoggedMechanismLigament2d(
                "Wrist Measured", Units.inchesToMeters(12.63), 0.0, 8.0, new Color8Bit(Color.kFirstRed)));

        intakeMeasured = intakeRoot.append(new LoggedMechanismLigament2d(
                "Intake Measured", Units.inchesToMeters(20.0), 0.0, 8.0, new Color8Bit(Color.kYellow)));
    }

    public void update() {
        double elevatorTravel = robot.getElevator().inputs.position[0];
        double elevatorHeight = elevatorToGround + elevatorTravel;

        elevatorMeasured.setLength(startingElevatorHeight + elevatorHeight);

        wristMeasured.setAngle(robot.getWrist().inputs.position);
        intakeMeasured.setAngle(robot.getIntakePivot().inputs.position);

        Logger.recordOutput("Superstructure/2D", mechanism2d);
    }
}
