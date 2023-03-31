package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Components;
import frc.robot.Constants;

public abstract class AutonomousBase {
    protected Components components;
    protected double startTime;
    protected double currentTime;
    protected double targetRollerSpeed;

    public AutonomousBase(Components components) {
        this.components = components;
    }

    public void init() {
        this.startTime = Timer.getFPGATimestamp();

        switch (components.targetChooser.getSelected()) {
            case Components.kLow:
                targetRollerSpeed = Constants.autoConstants.shooting.lowTargetSpeed;
                break;
            case Components.kMid:
                targetRollerSpeed = Constants.autoConstants.shooting.midTargetSpeed;
                break;
            default:
                targetRollerSpeed = Constants.autoConstants.shooting.lowTargetSpeed;
                break;
        }
        
        components.encoder.setPosition(0);
        components.gyro.reset();
    }

    public void periodic() {
        this.currentTime = Timer.getFPGATimestamp();
    }
}