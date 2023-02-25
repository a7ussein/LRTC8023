package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {


  // Auto Stuff:
  private static final String kDefaultAuto = "Nothing Auto";
  private static final String kDriveForward10Seconds = "Try Kick The Robot";
  private static final String kDriveForwardAndBalance = "Drive Forward and balance";
  private static final String kDepositAndDriveForward = "DepositCupeAndDriveForward";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
 // Variable Declarations for kick the bot auto!
  double leftSlow = 0.24;
  double rightSlow = -0.24;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25;

  // Inputs
  private ADIS16470_IMU gyro = new ADIS16470_IMU();

 // Outputs
 private CANSparkMax leftFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
 private CANSparkMax leftBackMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
 private CANSparkMax rightFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
 private CANSparkMax rightBackMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
 
 MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
 MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

 DifferentialDrive drive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

 
 RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
 RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();
 
// Intake Motors 
  private WPI_VictorSPX rollerMotor = new WPI_VictorSPX(5);
  private WPI_VictorSPX raisingMotor = new WPI_VictorSPX(6);

 //Unit Conversion 
 private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;

 public double getAverageEncoderDistance(){
  return ((leftEncoder.getPosition() * kDriveTick2Feet) + (rightEncoder.getPosition() * kDriveTick2Feet))/2;
 }

   // Controllers
private XboxController driveController = new XboxController(0);
private XboxController intakeController = new XboxController(1);

  @Override
  public void robotInit() {
    //Auto stuff:
    m_chooser.setDefaultOption("DoNothing", kDefaultAuto);
    m_chooser.addOption("TryToKickTheRbot", kDriveForward10Seconds);
    m_chooser.addOption("DriveForwardAndBalance", kDriveForwardAndBalance);
    m_chooser.addOption("DepositAndDriveForward", kDepositAndDriveForward);
    SmartDashboard.putData("Auto choices", m_chooser);

    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults(); 
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();

    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);


    gyro.reset();
    gyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left encoder value in feets", leftEncoder.getPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right encoder value in meters", rightEncoder.getPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("left Encoder Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right Encoder velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("YAW angle", gyro.getAngle());
    SmartDashboard.putNumber("Imu Turn Rate", gyro.getRate());
  }

  @Override
  public void autonomousInit() {
    // Auto Stuff:
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    gyro.reset();
    gyro.calibrate();
  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftEncoder.getPosition() * kDriveTick2Feet;
    double rightPosition = rightEncoder.getPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) /2;

    switch (m_autoSelected) {
      case kDriveForward10Seconds:
        if (Math.abs(gyro.getAngle()) <= 3) {
          leftControllerGroup.set(leftSlow - (gyro.getAngle()/ 15));
          rightControllerGroup.set(rightSlow - (gyro.getAngle()/15));;
        }else if(Math.abs(gyro.getAngle()) < 10){
          if(gyro.getAngle() > 0){
            leftControllerGroup.set(leftSlow);
            rightControllerGroup.set(1.1*rightSlow);
          }else if(gyro.getAngle()<0){
            leftControllerGroup.set(1.1*leftSlow);
            rightControllerGroup.set(rightSlow);
          }
        }else{
          if(gyro.getAngle() > 0){
            while(gyro.getAngle() > 10 && isAutonomous()){
              leftControllerGroup.set(-rotateSpeed);
              rightControllerGroup.set(-rotateSpeed);
            }
            while(gyro.getAngle() > 0 && isAutonomous()){
              leftControllerGroup.set(-rotateSpeedSlow);
              rightControllerGroup.set(-rotateSpeedSlow);
            }
            while(gyro.getAngle() < 0 && isAutonomous()){
              leftControllerGroup.set(rotateSpeedSlow);
              rightControllerGroup.set(rotateSpeedSlow);
            }
          }else{
            while(gyro.getAngle() < -10 && isAutonomous()){
              leftControllerGroup.set(rotateSpeed);
              rightControllerGroup.set(rotateSpeed);
            }
            while(gyro.getAngle() < 0 && isAutonomous()){
              leftControllerGroup.set(rotateSpeedSlow);
              rightControllerGroup.set(rotateSpeedSlow);
            }
            while(gyro.getAngle() > 0 && isAutonomous()){
              leftControllerGroup.set(-rotateSpeed);
              rightControllerGroup.set(-rotateSpeed);
            }
          }
        }
        break;
      case kDriveForwardAndBalance:
        if(distance < 10){
          drive.tankDrive(0.6, 0.6);
        }else{
          drive.tankDrive(0, 0);
        }
        //AUTO TO BALANCE ON CHARGING STATION:
        double vAngle = gyro.getYComplementaryAngle();  // vAngle stands for vertical angle
        if(Math.abs(vAngle) > 2){
          drive.tankDrive(-0.5, -0.5);
        }
        if(Math.abs(vAngle) < -2){
          drive.tankDrive(0.5, 0.5);
        }
        break;
      case kDepositAndDriveForward:
        rollerMotor.set(0.5);
        if(gyro.getAngle() < 1){
          drive.tankDrive(-0.6, 0.6);
        }else{
          drive.tankDrive(0, 0);
        }
        if(distance < 10){
          drive.tankDrive(0.6, 0.6);
        }else{
          drive.tankDrive(0, 0);
        }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    enableDrivingMotors(true);
    enableIntakeMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    System.out.println(Math.round(gyro.getAngle()));
 // drive controll
    double rightSpeed = -driveController.getRawAxis(1);  // for this axis: up is negative, down is positive
    double leftSpeed = -driveController.getRawAxis(5); 
    drive.tankDrive(rightSpeed *0.5, leftSpeed *0.5); //slowed speed down to 50%

    // intake Raising Controll
    double raisingPower = intakeController.getRawAxis(1);
    // deadBand 
    if(Math.abs(raisingPower) < 0.05){
      raisingPower = 0;
    }
    raisingMotor.set(raisingPower * 0.5);

    // intake Rollers control
    double rollersPower = 0;
    // press A if you want to pick up an object, and press Y if you want to shoot the object
    if(intakeController.getAButton() == true){
      rollersPower = 1;
    }else if(intakeController.getYButton() == true){
      rollersPower = -1;
    }

    rollerMotor.set(ControlMode.PercentOutput, rollersPower);
  }

  @Override
  public void disabledInit() {
    enableDrivingMotors(false);
    enableIntakeMotors(false);
  }

  @Override
  public void disabledPeriodic() {}


  private void enableDrivingMotors(boolean on){
    IdleMode dMotormode;
    if(on){
      dMotormode = IdleMode.kBrake;
    }else{
      dMotormode = IdleMode.kCoast;
    }
    leftFrontMotor.setIdleMode(dMotormode);
    leftBackMotor.setIdleMode(dMotormode); 
    rightFrontMotor.setIdleMode(dMotormode);
    rightBackMotor.setIdleMode(dMotormode); 
  }
  private void enableIntakeMotors(boolean on){
    NeutralMode iMotorMode;
    if(on){
      iMotorMode = NeutralMode.Brake;
    }else{
      iMotorMode = NeutralMode.Coast;
    }

    raisingMotor.setNeutralMode(iMotorMode);
    rollerMotor.setNeutralMode(iMotorMode);
  }
  //--------------------------

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
