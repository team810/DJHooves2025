package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PositionalControl implements SwerveControlIO{

    public PositionalControl(){

    }
    @Override
    public ChassisSpeeds getChassisSpeeds(Pose2d poseMeters) {
        return null;
    }
}
