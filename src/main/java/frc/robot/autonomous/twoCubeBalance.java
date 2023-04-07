package frc.robot.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Components;

public class twoCubeBalance extends AutonomousBase {
    public twoCubeBalance(Components components) {
        super(components);
    }

    private enum State {
        firstDeposit,
        starting,
        ascending,
        onFlat,
        descending,
        lowerArm,
        driveUntilCube,
        raiseArm,
        comeBackToTarget,
        startBalancing,
        balancing;
    }

    private State state;
    // private Double position;
    private final double encoder2inches = 1 / 8.46; // Unit Conversion

    private Double targetPosition;


    @Override
    public void init() {
        super.init();
        components.frontLimitSensor.get();
        components.backLimitSensor.get();
        components.cubeSensor.get();
        state = State.firstDeposit;
    }

    @Override
    public void periodic() {
        super.periodic();
        // boolean frontLimitSensor = components.frontLimitSensor.get();
        // boolean backLimitSensor = components.backLimitSensor.get();
        // boolean cubeSensor = components.cubeSensor.get();

        // vAngleTest is for YComplementaryAngle and I use it for autos that I am
        // testing,
        // I know I could just use vAngle that is in the kDriveForwardAndBalance but
        // I just don't want to miss around with it.
        double vAngleTest = components.gyro.getYComplementaryAngle();

        // rio is mounted backward
        vAngleTest = vAngleTest * -1;

        switch (state) {
            case firstDeposit:
                if (components.cubeSensor.get() == true) {
                    components.rollerMotor.set(this.targetRollerSpeed);
                } else {
                    components.rollerMotor.set(0);
                    System.out.println("This following state is over: " + state);
                    state = State.starting;
                }
                break;
            case starting:
                if (vAngleTest > 5) {
                    System.out.println("This following state is over: " + state);
                    state = State.ascending;
                } else {
                    components.drive.tankDrive(0.55, 0.55);
                }
                break;

            case ascending:
                if (vAngleTest < 0) {
                    System.out.println("This following state is over: " + state);
                    state = State.onFlat;
                } else {
                    components.drive.tankDrive(0.55, 0.55);
                }
                break;

            case onFlat:
                if (Math.abs(vAngleTest) > 5) {
                    System.out.println("The following state is over: " + state);
                    state = State.descending;
                } else {
                    components.drive.tankDrive(0.2, 0.2);
                }
                break;

            case descending:
                if (Math.abs(vAngleTest) < 2) {
                    // position = Math.abs(currentPosition);
                    System.out.println("The following state is over: " + state);
                    components.encoder.setPosition(0);
                    targetPosition = Math.abs(components.encoder.getPosition());
                    state = State.lowerArm;
                } else {
                    components.drive.tankDrive(0.2, 0.2);
                }
                break;

            case lowerArm:
                if (components.frontLimitSensor.get() == false) {
                    components.raisingMotor.set(ControlMode.PercentOutput, -0.5);
                } else {
                    components.raisingMotor.set(0);
                    System.out.println("The following state is over: " + state);
                    state = State.driveUntilCube;
                }
                break;
            // case driveUntilCube:
            // if((Math.abs(components.encoder.getPosition()) * encoder2inches) < 25 &&
            // components.cubeSensor.get() == false){
            // components.drive.tankDrive(0.6,0.6);
            // components.rollerMotor.set(-0.7);
            // System.out.println("State: " + state);
            // }else{
            // components.drive.tankDrive(0,0);
            // components.rollerMotor.set(ControlMode.PercentOutput, 0);
            // state = State.raiseArm;
            // }
            // break;

            case driveUntilCube:
                // Start timer when entering the state
                if (state != State.driveUntilCube) {
                    this.startTime = Timer.getFPGATimestamp();
                }

                // Check if timeout has occurred
                if ((Timer.getFPGATimestamp() - this.startTime) > 5.0) {
                    components.drive.tankDrive(0.0, 0.0);
                    components.rollerMotor.set(0.0);
                    state = State.raiseArm;
                    break;
                }

                if ((Math.abs(components.encoder.getPosition()) * encoder2inches) < 100 && components.cubeSensor.get() == false) {
                    components.drive.tankDrive(0.6, 0.6);
                    components.rollerMotor.set(-0.7);
                    System.out.println("State: " + state);
                } else {
                    components.drive.tankDrive(0, 0);
                    components.rollerMotor.set(0);
                    state = State.raiseArm;
                }
                break;

            case raiseArm:
                if (!components.backLimitSensor.get()) {
                    components.raisingMotor.set(ControlMode.PercentOutput, 0.6);
                    System.out.println("State: " + state);
                } else {
                    components.raisingMotor.set(0);
                    state = State.comeBackToTarget;
                }
                break;
            case comeBackToTarget:
                if (Math.abs(components.encoder.getPosition()) > targetPosition) {
                    components.raisingMotor.set(ControlMode.PercentOutput, 0);
                    components.drive.tankDrive(-0.6, -0.6);
                    System.out.println("State: " + state);
                } else {
                    state = State.balancing;
                }
                break;

            case balancing:
                if (vAngleTest > 2) {
                    components.drive.tankDrive(0.3, 0.3);
                    System.out.println("State: " + state);
                }
                if (vAngleTest < -2) {
                    components.drive.tankDrive(-0.3, -0.3);
                }
                break;

            default:
                break;
        }
    }
}