package frc.robot.subsystem.deflector;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DeflectorReal implements DeflectorIO{
    private final DoubleSolenoid deflector;

    public DeflectorReal()
    {
        deflector = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                DeflectorConstants.DEFLECTOR_FWD_CHANNEL,
                DeflectorConstants.DEFLECTOR_REV_CHANNEL
        );
    }
    @Override
    public void setState(DoubleSolenoid.Value value) {
        deflector.set(value);
    }

    @Override
    public void writePeriodic() {
//        Logger.recordOutput("Deflector/SolenoidValue", deflector.get());
    }
}
