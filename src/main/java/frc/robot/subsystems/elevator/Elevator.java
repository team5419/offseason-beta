package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;

    @Getter
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber stow =
            new LoggedTunableNumber("Elevator/Stow Height", kElevatorHeights.stow());
    private static final LoggedTunableNumber l1 = new LoggedTunableNumber("Elevator/Setpoints/L1", kElevatorHeights.l1());
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Elevator/Setpoints/L2", kElevatorHeights.l2());
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Elevator/Setpoints/L3", kElevatorHeights.l3());
    private static final LoggedTunableNumber l4 = new LoggedTunableNumber("Elevator/Setpoints/L4", kElevatorHeights.l4());

    @Getter
    @Setter
    @AutoLogOutput(key = "Elevator/Current Goal")
    private ElevatorGoal currentGoal = ElevatorGoal.STOW;

    public enum ElevatorGoal {
        IDLE(() -> 0),
        STOW(stow),
        L1(l1),
        L2(l2),
        L3(l3),
        L4(l4);

        @Getter
        private DoubleSupplier eleHeight;

        private ElevatorGoal(DoubleSupplier eleHeight) {
            this.eleHeight = eleHeight;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);

        Logger.recordOutput("Current Goal", currentGoal.toString());
        io.runPosition(currentGoal.getEleHeight().getAsDouble());
    }

    public void setDesiredLevel(ElevatorGoal goal) {}

    @AutoLogOutput(key = "Elevator/At Goal")
    public boolean atGoal() {
        return false;
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Units.Volts.per(Units.Seconds).of(0.5), // ramp rate: 0.5 V/s (gentle)
                    Units.Volts.of(2.0), // step voltage: 2V
                    Units.Seconds.of(10.0), // timeout
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                    (voltage) -> io.runVolts(voltage.in(Units.Volts)), // Apply voltage
                    (log) -> {
                        log.motor("elevator")
                                .voltage(Units.Volts.of(inputs.appliedVolts[0]))
                                .linearPosition(Units.Meter.of(inputs.position[0]))
                                .linearVelocity(Units.MetersPerSecond.of(inputs.velocityRotationsPerSecond[0]));
                    },
                    this));

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void runPosition(ElevatorGoal goal) {
        currentGoal = goal;
        // io.runPosition(currentGoal.getEleHeight().getAsDouble());
    }

    public void runPosition(double meters) {
        io.runPosition(meters);
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }
}
