package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PositionalControl implements SwerveControlIO{
    private final Pose2d target;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public PositionalControl(Pose2d target) {
        xController = new PIDController(0.05, 0, 0);
        yController = new PIDController(0.05, 0, 0);
        thetaController = new PIDController(0.05, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(.01);

        this.target = target;
    }
    @Override
    public ChassisSpeeds getChassisSpeeds(Pose2d currentPose) {
        return new ChassisSpeeds(
                xController.calculate(currentPose.getX(), target.getX()),
                yController.calculate(currentPose.getY(), target.getY()),
                thetaController.calculate(currentPose.getRotation().getRadians())
        );
    }
}
