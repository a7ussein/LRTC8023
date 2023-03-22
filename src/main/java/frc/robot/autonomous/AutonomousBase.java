package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;

public abstract class AutonomousBase {
    protected Components components;
    protected double startTime;
    protected double currentTime;

    public AutonomousBase(Components components) {
        this.components = components;
    }

    public void init() {
        this.startTime = Timer.getFPGATimestamp();
        
        components.encoder.setPosition(0);
        components.gyro.reset();
    }

    public void periodic() {
        this.currentTime = Timer.getFPGATimestamp();
    }
}