package frc.robot.subsystem.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class IntakeReal implements IntakeIO {

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;

    private double inputVoltage;

    public IntakeReal() {

        topMotor = new SparkMax(IntakeConstants.TOP_ID,
                SparkMax.MotorType.kBrushless);

        bottomMotor = new SparkMax(IntakeConstants.BOTTOM_ID,
                SparkMax.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(40);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        bottomMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config.inverted(true);
        topMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        topMotor.clearFaults();
        bottomMotor.clearFaults();


        inputVoltage = 0;
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        topMotor.set(inputVoltage);
        bottomMotor.set(inputVoltage);
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Intake/Top/Temperature", topMotor.getMotorTemperature());
        Logger.recordOutput("Intake/Top/CurrentDraw", topMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Top/MotorVoltage", topMotor.getBusVoltage());
        Logger.recordOutput("Intake/Top/InputVoltage", this.inputVoltage);

        Logger.recordOutput("Intake/Bottom/Temperature", bottomMotor.getMotorTemperature());
        Logger.recordOutput("Intake/Bottom/CurrentDraw", bottomMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Bottom/MotorVoltage", bottomMotor.getBusVoltage());
        Logger.recordOutput("Intake/Bottom/InputVoltage", this.inputVoltage);
    }
}
