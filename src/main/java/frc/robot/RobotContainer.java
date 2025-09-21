// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.DriveCommands;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.RumbleThread;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakepivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.IntakeRollerIOSim;
import frc.robot.subsystems.intakeroller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.outtakepivot.OuttakePivot;
import frc.robot.subsystems.outtakepivot.OuttakePivotIO;
import frc.robot.subsystems.outtakepivot.OuttakePivotIOSim;
import frc.robot.subsystems.outtakepivot.OuttakePivotIOTalonFX;
import frc.robot.subsystems.outtakeroller.OuttakeRoller;
import frc.robot.subsystems.outtakeroller.OuttakeRollerIO;
import frc.robot.subsystems.outtakeroller.OuttakeRollerIOSim;
import frc.robot.subsystems.outtakeroller.OuttakeRollerIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import java.io.File;
import lombok.Getter;

public class RobotContainer {

    @Getter
    private CommandXboxController driver = new CommandXboxController(Ports.kDriverPort);

    @Getter
    private CommandXboxController operator = new CommandXboxController(Ports.kOperatorPort);

    @Getter
    private AprilTagVision aprilTagVision;

    @Getter
    private Elevator elevator;

    @Getter
    private IntakePivot intakePivot;

    @Getter
    private IntakeRoller intakeRoller;

    @Getter
    private OuttakePivot outtakePivot;

    @Getter
    private OuttakeRoller outtakeRoller;

    @Getter
    private Swerve swerve;

    @Getter
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Get driver station to stop
        DriverStation.silenceJoystickConnectionWarning(true);

        buildRobot();
        configNamedCommands();

        autoChooser = buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureDefaultCommands();
        if (GlobalConstants.kDevMode) {
            configureDevBindings();
        } else {
            configureDriverBindings();
            configureOperatorBindings();
        }

        RumbleThread.getInstance().bindControllers(driver, operator);
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(DriveCommands.joystickDrive(
                swerve,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.leftBumper().getAsBoolean()));
    }

    private void configureDriverBindings() {
        driver.start(); // ! Unbound
        driver.back(); // ! Unbound

        driver.a(); // ! Unbound
        driver.b(); // ! Unbound
        driver.x(); // ! Unbound
        driver.y(); // ! Unbound

        driver.povUp(); // ! Unbound
        driver.povDown(); // ! Unbound
        driver.povLeft(); // ! Unbound
        driver.povRight(); // ! Unbound

        driver.leftBumper(); // ! Unbound
        driver.rightBumper(); // ! Unbound

        driver.leftTrigger(0.1); // ! Unbound
        driver.rightTrigger(0.1); // ! Unbound
    }

    private void configureOperatorBindings() {
        operator.start(); // ! Unbound
        operator.back(); // ! Unbound

        operator.a(); // ! Unbound
        operator.b(); // ! Unbound
        operator.x(); // ! Unbound
        operator.y(); // ! Unbound

        operator.povUp(); // ! Unbound
        operator.povDown(); // ! Unbound
        operator.povLeft(); // ! Unbound
        operator.povRight(); // ! Unbound

        operator.leftBumper(); // ! Unbound
        operator.rightBumper(); // ! Unbound

        operator.leftTrigger(0.1); // ! Unbound
        operator.rightTrigger(0.1); // ! Unbound
    }

    /**
     * Use this function to test new features without
     * changing the current button bindings
     */
    private void configureDevBindings() {}

    private void buildRobot() {
        Elevator tempElevator = null;
        IntakePivot tempIntakePivot = null;
        IntakeRoller tempIntakeRoller = null;
        OuttakePivot tempOuttakePivot = null;
        OuttakeRoller tempOuttakeRoller = null;
        Swerve tempSwerve = null;

        if (GlobalConstants.getMode() == GlobalConstants.Mode.REPLAY) return;
        switch (GlobalConstants.getRobotType()) {
            case BETA -> {
                tempElevator = new Elevator(new ElevatorIOTalonFX());
                tempIntakePivot = new IntakePivot(new IntakePivotIOTalonFX());
                tempIntakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
                tempOuttakePivot = new OuttakePivot(new OuttakePivotIOTalonFX());
                tempOuttakeRoller = new OuttakeRoller(new OuttakeRollerIOTalonFX());
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
            }

            case SIMBOT -> {
                tempElevator = new Elevator(new ElevatorIOSim());
                tempIntakePivot = new IntakePivot(new IntakePivotIOSim());
                tempIntakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                tempOuttakePivot = new OuttakePivot(new OuttakePivotIOSim());
                tempOuttakeRoller = new OuttakeRoller(new OuttakeRollerIOSim());
                tempSwerve = new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackRight()));
            }
        }

        if (tempSwerve == null)
            tempSwerve = new Swerve(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        if (tempElevator == null) tempElevator = new Elevator(new ElevatorIO() {});
        if (tempIntakePivot == null) tempIntakePivot = new IntakePivot(new IntakePivotIO() {});
        if (tempIntakeRoller == null) tempIntakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        if (tempOuttakePivot == null) tempOuttakePivot = new OuttakePivot(new OuttakePivotIO() {});
        if (tempOuttakeRoller == null) tempOuttakeRoller = new OuttakeRoller(new OuttakeRollerIO() {});

        swerve = tempSwerve;
        elevator = tempElevator;
        intakePivot = tempIntakePivot;
        intakeRoller = tempIntakeRoller;
        outtakePivot = tempOuttakePivot;
        outtakeRoller = tempOuttakeRoller;
    }

    /** Adds named commands to pathplanner */
    private void configNamedCommands() {
        NamedCommands.registerCommand(
                "Record Time", new InstantCommand(() -> RobotState.getInstance().setAutoFinished(true)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do nothing", new InstantCommand());
        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        for (File f : autosDir.listFiles()) {
            if (!f.isDirectory()) {
                String fileName[] = f.getName().split("\\.");
                String autoName = fileName[0];
                chooser.addOption(
                        autoName, AutoBuilder.buildAuto(autoName).beforeStarting(() -> System.out.println(autoName)));
            }
        }
        return chooser;
    }
}
