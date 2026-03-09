package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleAbsoluteEncoder implements AbsoluteEncoderInterface {

    private DutyCycleEncoder dutyCycleEncoder;
    public DutyCycleAbsoluteEncoder(int DIOPin) {
        this.dutyCycleEncoder = new DutyCycleEncoder(DIOPin);
    }
    @Override
    public void setInverted(boolean inverted) {
        this.dutyCycleEncoder.setInverted(inverted);
    }

    @Override
    public double get() {
        return this.dutyCycleEncoder.get();    
    }
}
