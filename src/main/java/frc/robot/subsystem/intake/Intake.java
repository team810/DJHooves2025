package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.Logger;

public class Intake {
    private final static Intake INSTANCE = new Intake();
    public static Intake getInstance() {return INSTANCE;}

    private final IntakeIO intake;
    private IntakeStates state;

    public Intake() {

        intake = new IntakeReal();
        state = IntakeStates.off;

    }


    public void readPeriodic() {
        intake.readPeriodic();
    }

    public void writePeriodic() {

        intake.writePeriodic();
        Logger.recordOutput("Intake State", state.toString());
    }


    public void setState(IntakeStates state) {
        this.state = state;
        switch (state)
        {
            case fwd -> {
                intake.setVoltage(IntakeConstants.INTAKE_MAX_SPEED * 12);
            }
            case rev -> {
                intake.setVoltage(-IntakeConstants.INTAKE_MAX_SPEED * 12);
            }
            case fire -> {
                intake.setVoltage(IntakeConstants.INTAKE_SHOOT_SPEED * 12);
            }
            case off -> {
                intake.setVoltage(0);
            }
        }
    }

}

