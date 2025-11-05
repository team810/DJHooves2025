package frc.robot.subsystem.deflector;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.MechanismState;
import org.littletonrobotics.junction.Logger;

public class Deflector {
    private final DeflectorIO deflector;
    private MechanismState deflectorState;

    public Deflector() {
        deflector = new DeflectorReal();

        deflectorState = MechanismState.stored;
        deflector.setState(DoubleSolenoid.Value.kReverse);
    }

    public void readPeriodic() {

    }

    public void writePeriodic() {
        Logger.recordOutput("Deflector/MechanismState", deflectorState);
        deflector.writePeriodic();
    }

    public MechanismState getDeflectorState() {
        return deflectorState;
    }

    public void setDeflectorState(MechanismState deflectorState) {
        this.deflectorState = deflectorState;
        if (deflectorState == MechanismState.deployed) {
            deflector.setState(DoubleSolenoid.Value.kForward);
        } else if (deflectorState == MechanismState.stored) {
            deflector.setState(DoubleSolenoid.Value.kReverse);
        }
    }

    public void toggleDeflectorState() {
        if (deflectorState == MechanismState.deployed) {
            deflector.setState(DoubleSolenoid.Value.kForward);
            deflectorState = MechanismState.stored;
        } else if (deflectorState == MechanismState.stored) {
            deflector.setState(DoubleSolenoid.Value.kReverse);
            deflectorState = MechanismState.deployed;
        }
    }

}

