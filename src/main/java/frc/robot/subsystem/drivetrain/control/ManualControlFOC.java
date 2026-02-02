package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Field oriented control
 */
public class ManualControlFOC implements  SwerveControlIO{
    public double HorizontalVelocity;
    public double VerticalVelocity;
    public double Omega;

    public ManualControlFOC(double horizontalVelocity, double verticalVelocity, double omega){
        this.HorizontalVelocity = horizontalVelocity;
        this.VerticalVelocity = verticalVelocity;
        this.Omega = omega;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds(Pose2d poseMeters) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                HorizontalVelocity,
                VerticalVelocity,
                Omega
        );
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseMeters.getRotation());
        return speeds;
    }
}
