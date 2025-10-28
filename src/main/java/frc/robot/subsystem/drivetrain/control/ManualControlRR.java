package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ManualControlRR implements SwerveControlIO{
    public double HorizontalVelocity;
    public double VerticalVelocity;
    public double Omega;

    public ManualControlRR(double horizontalVelocity, double verticalVelocity, double omega){
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
        return speeds;
    }
}
