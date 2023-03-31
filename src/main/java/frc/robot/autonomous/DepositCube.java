package frc.robot.autonomous;

import frc.robot.Components;

public class DepositCube extends AutonomousBase {

    public DepositCube(Components components) {
        super(components);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (this.currentTime - this.startTime < 1) {
            components.rollerMotor.set(this.targetRollerSpeed);
        } else {
            components.rollerMotor.set(0);
        }
    }
}