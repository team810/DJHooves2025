package frc.robot.subsystem.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {
    private final SparkMax topMotor;
    private final SparkMax bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final SparkClosedLoopController topController;
    private final SparkClosedLoopController bottomController;

    private double topTargetRPM;
    private double bottomTargetRPM;


    public ShooterReal() {
        topMotor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, SparkBase.MotorType.kBrushless);
        bottomMotor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(40);
        config.voltageCompensation(12);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);

        config.closedLoop.p(0.00006);
        config.closedLoop.i(0);
        config.closedLoop.d(0);
        config.closedLoop.velocityFF(.000185);

        bottomMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config.inverted(true);
        topMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        topMotor.clearFaults();
        bottomMotor.clearFaults();

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topController = topMotor.getClosedLoopController();
        bottomController = bottomMotor.getClosedLoopController();
    }
    @Override
    public void readPeriodic() {
        Logger.recordOutput("Shooter/Top/CurrentDraw", topMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Top/Temperature", topMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Top/Velocity", topEncoder.getVelocity());
        Logger.recordOutput("Shooter/Top/TargetVelocity", topTargetRPM);

        Logger.recordOutput("Shooter/Bottom/CurrentDraw", bottomMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Bottom/Temperature", bottomMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Bottom/Velocity", bottomEncoder.getVelocity());
        Logger.recordOutput("Shooter/Bottom/TargetVelocity", bottomTargetRPM);
    }

    @Override
    public void writePeriodic() {
        topController.setReference(topTargetRPM, SparkBase.ControlType.kVelocity);
        bottomController.setReference(bottomTargetRPM, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void setTopTargetRPM(double targetRPM) {
        this.topTargetRPM = targetRPM;
    }

    @Override
    public void setBottomTargetRPM(double targetRPM) {
        this.bottomTargetRPM = targetRPM;
    }


}
