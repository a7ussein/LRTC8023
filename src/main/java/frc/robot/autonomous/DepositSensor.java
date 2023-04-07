package frc.robot.autonomous;
import frc.robot.Components;


public class DepositSensor extends AutonomousBase{
    public DepositSensor(Components components){
        super(components);
    }

    @Override
    public void init(){
        super.init();
        components.cubeSensor.get();
    }

    @Override
    public void periodic(){
        super.periodic();

        if(components.cubeSensor.get()){
            components.rollerMotor.set(0.5);
        }else{
            components.rollerMotor.set(-0.5);
        }
    }
    
}
