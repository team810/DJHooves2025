package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class DrivetrainConstants {
    public static final String CANBUS = "drive train";
    public static final int GYRO_ID = 9;

    public static final Distance WHEEL_RADIUS = Distance.ofBaseUnits(2.0, Units.Inches);

    public static final double STEER_KP = 5;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0;
    public static final double STEER_MAX_OUTPUT = 12;

    public static final double WHEEL_BASE_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(24); // measure of FL wheel to FR wheel or BL wheel to BR wheel
    public static final double WHEEL_BASE_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(24); // measure of FL wheel to BL wheel// or FR wheel to BR wheel
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0)
    );

    public static int getDriveID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return 1;
            }
            case FrontRight -> {
                return 2;
            }
            case BackLeft -> {
                return 3;
            }
            case BackRight -> {
                return 4;
            }
        }
        return 0;
    }

    public static int getSteerID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return 8;
            }
            case FrontRight -> {
                return 6;
            }
            case BackLeft -> {
                return 5;
            }
            case BackRight -> {
                return 7;
            }
        }
        return 0;
    }

    public static int getEncoderID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return 5;
            }
            case FrontRight -> {
                return 6;
            }
            case BackLeft -> {
                return 7;
            }
            case BackRight -> {
                return 8;
            }
        }
        return 0;
    }

    public static String getID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return "FrontLeft";
            }
            case FrontRight -> {
                return "FrontRight";
            }
            case BackLeft -> {
                return "BackLeft";
            }
            case BackRight -> {
                return "BackRight";
            }
        }
        return "";
    }
    public static TalonFXConfiguration getDriveConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        //Motor Operating Mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Sensor Config
        config.Feedback.SensorToMechanismRatio = 5.9;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Voltage Config
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        // Audio Config
        config.Audio.BeepOnConfig = true;
        config.Audio.BeepOnBoot = true;
        config.Audio.AllowMusicDurDisable = true;
        // Slot 0 config
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = .124; // FIXME
        config.Slot0.kA = 0;
        config.Slot0.kG = 0;

        return config;
    }

}
