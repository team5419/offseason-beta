package frc.robot.constants;

public class Ports {

    // controllers
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    // drivetrain
    public static final int kFrontLeftDriveID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 10;
                default -> 0;
            };
    public static final int kFrontLeftSteerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 20;
                default -> 0;
            };
    public static final int kFrontLeftEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 30;
                default -> 0;
            };

    public static final int kFrontRightDriveID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 11;
                default -> 0;
            };
    public static final int kFrontRightSteerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 21;
                default -> 0;
            };
    public static final int kFrontRightEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 31;
                default -> 0;
            };

    public static final int kBackLeftDriveID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 13;
                default -> 0;
            };
    public static final int kBackLeftSteerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 23;
                default -> 0;
            };
    public static final int kBackLeftEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 33;
                default -> 0;
            };

    public static final int kBackRightDriveID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 12;
                default -> 0;
            };
    public static final int kBackRightSteerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 22;
                default -> 0;
            };
    public static final int kBackRightEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 32;
                default -> 0;
            };

    public static final int kPigeonID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 51;
                default -> 0;
            };

    public static final int kIntakeBeambreakPort =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 0; // TODO FIND VALUES
                default -> 0;
            };
    public static final int kHandoffBeambreakPort =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 1; // TODO FIND VALUES
                default -> 0;
            };
    public static final int kEndBeambreakPort =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 2; // TODO FIND VALUES
                default -> 0;
            };

    public static final int kLedPort =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 3; // TODO FIND VALUES
                default -> 0;
            };

    public static final int kEleLeaderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 19;
                default -> 0;
            };

    public static final int kEleFollowerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 18;
                default -> 0;
            };

    public static final int kIntakePivotID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 59;
                default -> 0;
            };

    public static final int kIntakeRollerLeaderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 56;
                default -> 0;
            };

    public static final int kIntakeRollerFollowerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 55;
                default -> 0;
            };

    public static final int kWristID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 49;
                default -> 0;
            };

    public static final int kEndEffectorLeaderID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 46;
                default -> 0;
            };

    public static final int kEndEffectorFollowerID =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 55;
                default -> 0;
            };
}
