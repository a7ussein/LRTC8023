package frc.robot.autonomous;

public class DepositAndDriveForward extends AutonomousBase {
    public DepositAndDriveForward() {
    }

    @Override
    public void init() {
        super.init();

        if(this.currentTime - this.startTime < 1){
            components.rollerMotor.set(.7);
        } else {
            components.rollerMotor.set(0);
        }

        if ((Math.abs(components.getEncoder().getPosition()) / 8.46) < 8.5) {
          drive.tankDrive(0.3, 0.3);
        } else {
          drive.tankDrive(0, 0);
      }
    }
}