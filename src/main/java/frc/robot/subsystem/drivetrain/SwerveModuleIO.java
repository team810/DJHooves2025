package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    public void readPeriodic();
    public void writePeriodic();
    /**
     * @param state The target module state
     */
    public void setState(SwerveModuleState state);
    /**
     * @return Reads sensor data to determine the current swerve module state
     */
    public SwerveModuleState getCurrentState();

    public SwerveModulePosition getModulePosition();
}
