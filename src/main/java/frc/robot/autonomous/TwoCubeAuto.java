/*
* this auto is going to deposit a cube on the low scoring area
* drive forward 8.5 wheel rotations
* check if the robot arm is backward
*      if it is backward it will make it forward
*      then it will check if there is a cube in the intake
*      if there is no cube in the intake, it will run the intake motor until there is a cube in the intake
*  
* then it will check if the arm is backward or not, if it is not backward it will put it backward
* then it will drive back to the starting point and it will deposit the cube to the middle run
*/
package frc.robot.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Components;


public class TwoCubeAuto extends AutonomousBase {
    private enum State{
        firstCubeDeposit,
        lowerArm,
        driveForwardUntilCube,
        raiseArm,
        driveBackward,
        secondCubeDeposit,
        Finished;
    }

    private State currentState;
    private final double encoder2inches = 1 / 8.46; // Unit Conversion

    public TwoCubeAuto(Components components) {
        super(components);
        currentState = State.firstCubeDeposit;
    }

    @Override
    public void periodic() {
        super.periodic();

        switch(currentState){
            case firstCubeDeposit:
                if(this.currentTime - this.startTime < 1){
                    components.rollerMotor.set(ControlMode.PercentOutput, 0.5);
                }else{
                    components.rollerMotor.set(ControlMode.PercentOutput, 0);
                    currentState = State.lowerArm;
                }
            break;
            
            case lowerArm:
                if(!components.frontLimitSensor.get()){
                    components.raisingMotor.set(ControlMode.PercentOutput, 0.6);
                }else{
                    components.raisingMotor.set(ControlMode.PercentOutput, 0);
                    currentState = State.driveForwardUntilCube;
                }
            break;

            case driveForwardUntilCube:
                if((Math.abs(components.encoder.getPosition()) * encoder2inches) < (236 * encoder2inches) && !components.cubeSensor.get()){
                    components.drive.tankDrive(0.3, 0.3);
                    components.rollerMotor.set(ControlMode.PercentOutput, -0.7);
                }else{
                    components.rollerMotor.set(ControlMode.PercentOutput, 0);
                    components.drive.tankDrive(0, 0);
                    currentState = State.raiseArm;
                }
            break;

            case raiseArm:
                if (components.backLimitSensor.get()){
                    components.raisingMotor.set(ControlMode.PercentOutput, 0);
                }else{
                    components.raisingMotor.set(ControlMode.PercentOutput, -0.6);
                    currentState = State.driveBackward;
                }
            break;

            case driveBackward:
                if((Math.abs (components.encoder.getPosition()) / 8.46) > 0){
                    components.drive.tankDrive(-0.3, -0.3);
                }else{
                    components.drive.tankDrive(0, 0);
                    currentState = State.secondCubeDeposit;
                }
            break;
                
            case secondCubeDeposit:
                if(components.cubeSensor.get()){
                    components.rollerMotor.set(ControlMode.PercentOutput, 1);
                }else{
                    currentState = State.Finished;
                }
            break;

            case Finished:
                components.drive.tankDrive(0, 0);
                components.rollerMotor.set(ControlMode.PercentOutput, 0);
                components.raisingMotor.set(ControlMode.PercentOutput, 0);
            break;

            default:
                break;
        }



    }
}