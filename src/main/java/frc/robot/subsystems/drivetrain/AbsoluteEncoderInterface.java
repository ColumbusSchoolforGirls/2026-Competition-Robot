package frc.robot.subsystems.drivetrain;

public interface AbsoluteEncoderInterface {
    public void setInverted(boolean inverted);
    
    /*
     * Get the encoder value since the last reset.
        This is reported in rotations since the last reset.
        Returns:
        the encoder value in rotations
     */
    public double get();
    
}
