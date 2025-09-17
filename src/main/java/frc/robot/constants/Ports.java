package frc.robot.constants;

public class Ports {

    // controllers
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    // drivetrain
    public static final int kFrontLeftDriveID = 0; // TODO, GET PORTS
    public static final int kFrontLeftSteerID = 0; // TODO, GET PORTS
    public static final int kFrontLeftEncoderID = 0; // TODO, GET PORTS

    public static final int kFrontRightDriveID = 0; // TODO, GET PORTS
    public static final int kFrontRightSteerID = 0; // TODO, GET PORTS
    public static final int kFrontRightEncoderID = 0; // TODO, GET PORTS

    public static final int kBackLeftDriveID = 0; // TODO, GET PORTS
    public static final int kBackLeftSteerID = 0; // TODO, GET PORTS
    public static final int kBackLeftEncoderID = 0; // TODO, GET PORTS

    public static final int kBackRightDriveID = 0; // TODO, GET PORTS
    public static final int kBackRightSteerID = 0; // TODO, GET PORTS
    public static final int kBackRightEncoderID = 0; // TODO, GET PORTS

    public static final int kLedPort = 9;

    public static final int kPigeonID = 51;

    public static final int kRollerID = 0; // TODO, GET PORTS

    public static final int kPivotID =
            switch (GlobalConstants.getRobotType()) {
                default -> 27;
            };

    public static final int kEleLeaderID =
            switch (GlobalConstants.getRobotType()) {
                default -> 19;
            };

    public static final int kEleFollowerID =
            switch (GlobalConstants.getRobotType()) {
                default -> 18;
            };

    public static final int kClimbLeaderID =
            switch (GlobalConstants.getRobotType()) {
                default -> 28;
            };

    public static final int kClimbFollowerID =
            switch (GlobalConstants.getRobotType()) {
                default -> 29;
            };

    public static final int kBeamBreakPort = 0;
}