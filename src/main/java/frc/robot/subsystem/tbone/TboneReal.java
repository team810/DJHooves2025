package frc.robot.subsystem.tbone;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class TboneReal implements TBoneIO {

    private SparkMax motor;

    private double inputVoltage;

    private RelativeEncoder encoder;

    public TboneReal() {

        motor = new SparkMax(TboneConstants.TBONE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(true);
        config.voltageCompensation(12);
        config.smartCurrentLimit(40);

        motor.clearFaults();


        encoder = motor.getAlternateEncoder();

        inputVoltage = 0;
        setVoltage(0);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        motor.set(inputVoltage);
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("T-Bone/Position",encoder.getPosition());
        Logger.recordOutput("T-Bone/Voltage", inputVoltage);

    }
}
