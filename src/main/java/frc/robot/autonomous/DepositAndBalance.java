package frc.robot.autonomous;

import frc.robot.Components;

public class DepositAndBalance extends AutonomousBase {
    public DepositAndBalance(Components components) {
        super(components);
    }

    private Boolean starting;
    private Boolean descending;
    private Boolean onFlat;
    private Boolean ascending;
    private Boolean exitingRamp;
    private Boolean startBalancing;
    private Boolean balancing;
    private Double position;

    @Override
    public void init() {
        super.init();

        starting = true;
        descending = false;
        onFlat = false;
        ascending = false;
        exitingRamp = false;
        startBalancing = false;
        balancing = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        // vAngleTest is for YComplementaryAngle and I use it for autos that I am
        // testing,
        // I know I could just use vAngle that is in the kDriveForwardAndBalance but
        // I just don't want to miss around with it.
        double vAngleTest = components.gyro.getYComplementaryAngle();
        double currentPosition = components.encoder.getPosition();

        // rio is mounted backward
        vAngleTest = vAngleTest * -1;

        if (this.currentTime - this.startTime < 1) {
            components.rollerMotor.set(this.targetRollerSpeed);
        } else {
            components.rollerMotor.set(0);
        }

        // BALANCE

        if (starting && vAngleTest > 5) {
            ascending = true;
            starting = false;
        }

        if (ascending && vAngleTest < 0) {
            ascending = false;
            onFlat = true;
        }

        if (onFlat && Math.abs(vAngleTest) > 5) {
            onFlat = false;
            descending = true;
        }

        if (descending && Math.abs(vAngleTest) < 2) {
            descending = false;
            exitingRamp = true;
            position = Math.abs(currentPosition);
        }

        if (starting || ascending) {
            components.drive.tankDrive(0.55, 0.55);
        }

        if (onFlat || descending) {
            components.drive.tankDrive(0.2, 0.2);
        }

        if (exitingRamp) {
            if (Math.abs(currentPosition) < position + 3) {
                components.drive.tankDrive(0.3, 0.3);
            } else {
                exitingRamp = false;
                startBalancing = true;
                position = currentPosition - 20;
            }
        }

        if (startBalancing) {
            if (currentPosition > position) {
                components.drive.tankDrive(-0.6, -0.6);
            } else {
                startBalancing = false;
                balancing = true;
            }
        }

        if (balancing) {
            if (vAngleTest > 2) {
                components.drive.tankDrive(0.27, 0.27);
            }
            if (vAngleTest < -2) {
                components.drive.tankDrive(-0.27, -0.27);
            }
        }
    }
}