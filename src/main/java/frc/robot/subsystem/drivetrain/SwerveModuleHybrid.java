package frc.robot.subsystem.drivetrain;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve module with a neo steer motor and a kraken drive motor
 */
public class SwerveModuleHybrid implements SwerveModuleIO {
    private final SwerveModuleID id;
    private final String idString;

    private final TalonFX driveMotor;
    // Drive motor encoder data
    private final StatusSignal<Angle> driveAngularPosition;
    private final StatusSignal<AngularVelocity> driveAngualrVelocity;
    // Drive motor data
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Current> driveAppliedCurrent;
    private final StatusSignal<Current> driveSupplyCurrent;
    // Drive control
    private final VelocityVoltage driveControl;
    private AngularVelocity driveTargetAngularVelocity;
    private LinearVelocity driveTargetLinearVelocity;
    // Drive data
    private Distance driveLinearPosition;
    private LinearVelocity driveLinearVelocity;

    private final SparkMax steerMotor;
    private final PIDController steerController;

    private final CANcoder steerEncoder;
    private final StatusSignal<Angle> thetaStatusSignal;

    private SwerveModuleState currentState;
    private SwerveModuleState targetState;

    private SwerveModulePosition modulePosition;

    public SwerveModuleHybrid(SwerveModuleID id) {
        this.id = id;
        idString = DrivetrainConstants.getID(id);

        driveMotor = new TalonFX(
                DrivetrainConstants.getDriveID(id),
                DrivetrainConstants.CANBUS
        );
        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig());

        driveAngularPosition = driveMotor.getPosition();
        driveAngualrVelocity = driveMotor.getVelocity();

        driveAppliedVoltage = driveMotor.getMotorVoltage();
        driveAppliedCurrent = driveMotor.getStatorCurrent();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();

        driveLinearPosition = Distance.ofBaseUnits(0,Units.Meter);
        driveLinearVelocity = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);

        driveControl = new VelocityVoltage(0);
        driveControl.Slot = 0;
        driveControl.EnableFOC = true;
        driveControl.LimitForwardMotion = false;
        driveControl.LimitReverseMotion = false;

        steerMotor = new SparkMax(DrivetrainConstants.getSteerID(id), SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        steerMotor.configure(config,SparkMax.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        steerEncoder = new CANcoder(DrivetrainConstants.getEncoderID(id), DrivetrainConstants.CANBUS);

        steerController = new PIDController(
                DrivetrainConstants.STEER_KP,
                DrivetrainConstants.STEER_KI,
                DrivetrainConstants.STEER_KD
        );
        steerController.enableContinuousInput(-Math.PI, Math.PI);

        thetaStatusSignal = steerEncoder.getAbsolutePosition();
        StatusSignal.setUpdateFrequencyForAll(250,
                driveAngularPosition,
                driveAngualrVelocity,
                driveAppliedVoltage,

                thetaStatusSignal
        );
        StatusSignal.setUpdateFrequencyForAll(25,
                driveAppliedCurrent,
                driveSupplyCurrent
        );


        targetState = new SwerveModuleState();
        currentState = targetState;

        modulePosition = new SwerveModulePosition();
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                driveAngularPosition,
                driveAngualrVelocity,
                driveAppliedVoltage,
                driveAppliedCurrent,
                driveSupplyCurrent,

                thetaStatusSignal
        );

        driveLinearPosition = Distance.ofBaseUnits(
                driveAngularPosition.getValue().in(Units.Radians) * DrivetrainConstants.WHEEL_RADIUS.in(Units.Meter),
                Units.Meters
        );
        driveLinearVelocity = LinearVelocity.ofBaseUnits(
                driveAngualrVelocity.getValue().in(Units.RadiansPerSecond) * DrivetrainConstants.WHEEL_RADIUS.in(Units.Meter),
                Units.MetersPerSecond
        );

        modulePosition = new SwerveModulePosition(
                driveLinearPosition,
                Rotation2d.fromRadians(thetaStatusSignal.getValue().in(Units.Radians))
        );

        Logger.recordOutput("Drivetrain/" + idString + "/CurrentState", currentState);
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/Rotations", driveAngularPosition.getValue().in(Units.Rotations));
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/CurrentRotationsPerSecond", driveAngualrVelocity.getValue().in(Units.RotationsPerSecond));
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AppliedVoltage", driveAppliedVoltage.getValue().in(Units.Volts));
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AppliedCurrent", driveAppliedCurrent.getValue().in(Units.Amps));
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/SupplyCurrent", driveSupplyCurrent.getValue().in(Units.Amps));
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/CurrentAngle", thetaStatusSignal.getValue().in(Units.Radians));
    }

    @Override
    public void writePeriodic() {
        // Optimize
        targetState.optimize(Rotation2d.fromRadians(thetaStatusSignal.getValue().in(Units.Radians)));
        targetState.cosineScale(Rotation2d.fromRadians(thetaStatusSignal.getValue().in(Units.Radians)));

        // Drive control
        AngularVelocity driveTargetVelocity = AngularVelocity.ofBaseUnits(
                targetState.speedMetersPerSecond / DrivetrainConstants.WHEEL_RADIUS.in(Units.Meters),
                Units.RotationsPerSecond
        );
        driveControl.Velocity = driveTargetVelocity.in(Units.RotationsPerSecond);
        driveMotor.setControl(driveControl);

        // Steer Control
        double steerOutput = MathUtil.clamp(
                MathUtil.applyDeadband(
                    steerController.calculate(thetaStatusSignal.getValue().in(Units.Radians), targetState.angle.getRadians()),
                    .1
                ),
                -DrivetrainConstants.STEER_MAX_OUTPUT,
                DrivetrainConstants.STEER_MAX_OUTPUT
        );

        Logger.recordOutput("Drivetrain/" + idString + "/TargetState", targetState);
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/TargetRotationsPerSecond", driveTargetVelocity.in(Units.RotationsPerSecond));
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/AppliedVoltage", steerOutput);
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/Target", targetState.angle.getRadians());
    }

    @Override
    public void setState(SwerveModuleState state) {
        this.targetState = state;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return modulePosition;
    }
}
