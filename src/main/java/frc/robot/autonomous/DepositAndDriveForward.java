package frc.robot.autonomous;

import frc.robot.Components;

public class DepositAndDriveForward extends AutonomousBase {

    public DepositAndDriveForward(Components components) {
        super(components);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (this.currentTime - this.startTime < 1) {
            components.rollerMotor.set(.7);
        } else {
            components.rollerMotor.set(0);
        }

        if ((Math.abs(components.encoder.getPosition()) / 8.46) < 8.5) {
            components.drive.tankDrive(0.3, 0.3);
        } else {
            components.drive.tankDrive(0, 0);
        }
    }
}