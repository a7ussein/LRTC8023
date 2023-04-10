package frc.robot.autonomous;

import frc.robot.Components;

public class raiseArm extends AutonomousBase {
    public raiseArm(Components components){
        super(components);
    }

    private boolean frontLimitSensor;
    private boolean backLimitSensor;

    @Override
    public void init(){
        super.init();
    }

    @Override
    public void periodic(){
        super.periodic();

        frontLimitSensor = components.frontLimitSensor.get();
        backLimitSensor = components.backLimitSensor.get();
            if(frontLimitSensor == true && backLimitSensor == false){
                components.raisingMotor.set(-0.6);
            }else{
                components.raisingMotor.set(0);
            }
    }

}
