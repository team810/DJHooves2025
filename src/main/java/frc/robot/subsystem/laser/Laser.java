package frc.robot.subsystem.laser;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Laser {
    private static final Laser INSTANCE = new Laser();
    public static Laser getInstance() {return INSTANCE;}

    private final LaserCan sensor;
    double distance = 0;

    private LaserState state;

    private Laser() {
        sensor = new LaserCan(LaserConstants.ID);

        if (Robot.isReal())
        {
            distance = sensor.getMeasurement().distance_mm;
        }else{
            distance = 0;
        }

        if (MathUtil.isNear(LaserConstants.EXPECTED_DISTANCE, distance, LaserConstants.TOLERANCE))
        {
            state = LaserState.Detected;
        }else{
            state = LaserState.Undetected;
        }
    }
    public LaserState getLaserState()
    {
        if (Robot.isReal())
        {
            distance = sensor.getMeasurement().distance_mm;
        }else{
            distance = 30;
        }
        if (MathUtil.isNear(LaserConstants.EXPECTED_DISTANCE, distance, LaserConstants.TOLERANCE))
        {
            state = LaserState.Detected;
        }else{
            state = LaserState.Undetected;
        }
        return state;
    }

    public void readPeriodic() {
        Logger.recordOutput("LaserSensor/Distance", distance);
        Logger.recordOutput("LaserSensor/State", state);
    }

}

