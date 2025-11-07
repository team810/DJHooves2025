package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.subsystem.drivetrain.control.SwerveControlIO;
import org.littletonrobotics.junction.Logger;

public class Drivetrain {
    private static Drivetrain INSTANCE = new Drivetrain();
    public static Drivetrain getInstance() {return INSTANCE;}

    private final Pigeon2 gyro;
    private final StatusSignal<Angle> yawSignal;

    private final SwerveModuleIO frontLeftModule;
    private final SwerveModuleIO frontRightModule;
    private final SwerveModuleIO backLeftModule;
    private final SwerveModuleIO backRightModule;

    private final SwerveDrivePoseEstimator estimator;

    private SwerveControlIO control;

    private Drivetrain() {
        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CANBUS);
        yawSignal = gyro.getYaw();
        yawSignal.setUpdateFrequency(250);

        frontLeftModule = new SwerveModuleHybrid(SwerveModuleID.FrontLeft);
        frontRightModule = new SwerveModuleHybrid(SwerveModuleID.FrontRight);
        backLeftModule = new SwerveModuleHybrid(SwerveModuleID.BackLeft);
        backRightModule = new SwerveModuleHybrid(SwerveModuleID.BackRight);


        estimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.SWERVE_KINEMATICS,
                Rotation2d.fromRadians(yawSignal.getValue().in(Units.Radians)),
                new SwerveModulePosition[]{
                        frontLeftModule.getModulePosition(),
                        frontRightModule.getModulePosition(),
                        backLeftModule.getModulePosition(),
                        backRightModule.getModulePosition()
                },
                new Pose2d()
        );
    }

    public void readPeriodic() {
        StatusSignal.refreshAll(yawSignal);
        estimator.update(
                Rotation2d.fromRadians(yawSignal.getValue().in(Units.Radians)),
                new SwerveModulePosition[]{
                        frontLeftModule.getModulePosition(),
                        frontRightModule.getModulePosition(),
                        backLeftModule.getModulePosition(),
                        backRightModule.getModulePosition()
                }
        );
        Logger.recordOutput("EstimatedPose", estimator.getEstimatedPosition());
    }

    public void writePeriodic() {
        ChassisSpeeds speeds = control.getChassisSpeeds(estimator.getEstimatedPosition());
        SwerveModuleState states[] = DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);

        Logger.recordOutput("Drivetrain/ChassisSpeeds", speeds);
    }

    public void setControl(SwerveControlIO control) {
        this.control = control;
    }

    public SwerveControlIO getControl() {
        return control;
    }
    public void setPose(Pose2d pose) {
        estimator.resetPose(pose);
    }
}
