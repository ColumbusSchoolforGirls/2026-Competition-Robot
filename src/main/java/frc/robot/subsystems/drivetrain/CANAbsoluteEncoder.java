package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class CANAbsoluteEncoder implements AbsoluteEncoderInterface {
    private CANcoder canEncoder;

    public CANAbsoluteEncoder(int canID) {
        this.canEncoder = new CANcoder(canID);
    }

    @Override
    public void setInverted(boolean inverted) {
        // TODO: figure out the correct configs
        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canEncoder.getConfigurator().apply(configs);
    }

    @Override
    public double get() {
        return canEncoder.getAbsolutePosition().getValueAsDouble();
    }
}
