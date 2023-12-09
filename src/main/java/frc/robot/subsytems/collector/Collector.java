package frc.robot.subsytems.collector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
    private final CollectorIO io;
    private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

    public Collector(CollectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Collector", inputs);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }
}
