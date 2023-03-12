package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class Robot extends TimedRobot {

  // Auto Stuff:
  private static final String kDefaultAuto = "Nothing Auto";
  private static final String kDriveForwardAndBalance = "Drive Forward and balance";
  private static final String kDepositAndDriveForward = "Deposit Cupe And Drive Forward";
  private static final String kDepositAndBalance = "Deposit Cube and Balance";
  private static final String kgsdDepositAndBalance = "GSD Deposit and Balance Code";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  // Inputs
  private ADIS16470_IMU gyro = new ADIS16470_IMU();


  // variable to keep track of time
  private double startTime;

  // SlewRateLimiter variable:
  SlewRateLimiter limiter = new SlewRateLimiter(0.5);

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

  DigitalInput frontLimitSensor = new DigitalInput(8);
  DigitalInput backLimitSensor = new DigitalInput(9);

  // Intake Motors
  private WPI_VictorSPX rollerMotor = new WPI_VictorSPX(5);
  private WPI_VictorSPX raisingMotor = new WPI_VictorSPX(6);

  // Unit Conversion
  // private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;

  // This should now disblay the number of wheel rotations.
  public double getAverageEncoderDistance() {
    return ((Math.abs(leftEncoder.getPosition()) + (rightEncoder.getPosition()) / 2)/8.46);
  }

  // Controllers
  private XboxController driveController = new XboxController(0);
  private XboxController intakeController = new XboxController(1);

  @Override
  public void robotInit() {
    // Auto stuff:
    m_chooser.setDefaultOption("DoNothing", kDefaultAuto);
    // m_chooser.addOption("DriveForwardAndBalance", kDriveForwardAndBalance); // don't need this to show on shuffle board.
    m_chooser.addOption("DepositAndDriveForward", kDepositAndDriveForward);
    m_chooser.addOption("DepositCubeAndBalance", kDepositAndBalance);
    m_chooser.addOption("gsdDepositBalance", kgsdDepositAndBalance);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Camera init:
    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(640, 480);
    // camera.setFPS(20);


    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();

    // Invertation Settings
    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void robotPeriodic() {
    double leftPosition = leftEncoder.getPosition();
    SmartDashboard.putNumber("Left Position", leftPosition);
    SmartDashboard.putNumber("Left encoder value", leftEncoder.getPosition() );
    SmartDashboard.putNumber("Right encoder value", rightEncoder.getPosition());
    SmartDashboard.putNumber("left Encoder Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right Encoder velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Average Encoder Distance in wheel rotations", getAverageEncoderDistance());
    SmartDashboard.putNumber("YComplementaryAngle", gyro.getYComplementaryAngle() * -1);
    SmartDashboard.putNumber("YAW angle", gyro.getAngle());
    SmartDashboard.putNumber("Imu Turn Rate", gyro.getRate());
    String msg = "Message";
    String ahmed = "Good Luck Kids";
    SmartDashboard.putString(msg, ahmed);
  }

  @Override
  public void autonomousInit() {
    // Auto Stuff:
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    startTime = Timer.getFPGATimestamp();

    leftEncoder.setPosition(0);
    gyro.reset();

    // Balancing auto booleans inits -- Don't play around with them
    m_starting = true;
    m_onRamp = false;
    m_descending = false;
    m_onFlat = false;
    m_ascending = false;
    m_exitingRamp = false;
    m_startBalancing = false;
    m_balancing = false;
  }
  
  // Balancing auto Booleans:
  private Boolean m_starting;
  private Boolean m_onRamp;
  private Boolean m_descending;
  private Boolean m_onFlat;
  private Boolean m_ascending;
  private Boolean m_exitingRamp;
  private Boolean m_startBalancing;
  private Boolean m_balancing;
  private Double m_position;

  @Override
  public void autonomousPeriodic() {
    // Don't play with these values cuz they are gonna affect the balancing auto
    double leftPosition = leftEncoder.getPosition();
    double rightPosition = rightEncoder.getPosition();
    double distance = (Math.abs(leftPosition) + Math.abs(rightPosition)) / 2;
    double vAngleTest = gyro.getYComplementaryAngle(); // vAngleTes is for YComplementartAngle and I use it for autos that I am testing, I know I could just use vAngle that is in the kDriveForwardAndBalance but I just don't want to miss around with it. 

    // timer related stuff:
    double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);
    
    switch (m_autoSelected) {
      // kDriveForwardAndBalance is the base code
      case kDriveForwardAndBalance:
      if(time - startTime < 5){
        rollerMotor.set(.7);
       }else{
        rollerMotor.set(0);
       }
      enableIntakeMotors(true);
       double vAngle = gyro.getYComplementaryAngle(); // vAngle stands for virticle angle AKA YComplementartAngle

        // rio is mounted backward
        vAngle = vAngle * -1;

        if (m_starting && vAngle > 5) {
          m_onRamp = true;
          m_ascending = true;
          m_starting = false;
        }

        if (m_ascending && vAngle < 0) {
          m_ascending = false;
          m_onFlat = true;
        }

        if (m_onFlat && Math.abs(vAngle) > 5) {
          m_onFlat = false;
          m_descending = true;
        }

        if (m_descending && Math.abs(vAngle) < 2) {
          m_descending = false;
          m_onRamp = false;
          m_exitingRamp = true;
          m_position = Math.abs(leftPosition);
        }

        if (m_starting || m_ascending) {
          drive.tankDrive(0.55, 0.55);
        }

        if (m_onFlat || m_descending) {
          drive.tankDrive(0.2, 0.2);
        }

        if (m_exitingRamp){
          if (Math.abs(leftPosition) < m_position + 4){
            drive.tankDrive(0.3, 0.3);
          }
          else {
            m_exitingRamp = false;
            m_startBalancing = true;
            //leftEncoder.setPosition(0);
            m_position = leftPosition - 20;
          }
        }
      
        if (m_startBalancing) {
          if (leftPosition > m_position) {
          //if (Math.abs(vAngle) > 10) {
            drive.tankDrive(-0.55, -0.55);
          } else {
            m_startBalancing = false;
            m_balancing = true;
          }
        }

        if (m_balancing) {
          if (vAngle > 2) {
            drive.tankDrive(0.3, 0.3);
          }
          if (vAngle < -2) {
            drive.tankDrive(-0.3, -0.3);
          }
        }
        break;
      case kDepositAndDriveForward:
       // shoot the cube out then drive forward for 8.5 wheel rotations
       if(time - startTime < 5){
        rollerMotor.set(.7);
       }else{
        rollerMotor.set(0);
       }
        if ((Math.abs(leftPosition)/ 8.46) < 8.5) {
          drive.tankDrive(0.3, 0.3);
        } else {
          drive.tankDrive(0, 0);
      }
        break;
      case kDepositAndBalance:
          /*
         * This auto is going to:
         * 1. eject the cube which takes about 2 seconds,
         * 2. balance which takes about 10 seconds 
         */
        if(time - startTime < 5){
          rollerMotor.set(.7);
         }else{
          rollerMotor.set(0);
         }
        // BALANCE 
        // rio is mounted backward
        vAngleTest = vAngleTest * -1;

      if (m_starting && vAngleTest > 5) {
        m_onRamp = true;
        m_ascending = true;
        m_starting = false;
      }

      if (m_ascending && vAngleTest < 0) {
        m_ascending = false;
        m_onFlat = true;
      }

      if (m_onFlat && Math.abs(vAngleTest) > 5) {
        m_onFlat = false;
        m_descending = true;
      }

      if (m_descending && Math.abs(vAngleTest) < 2) {
        m_descending = false;
        m_onRamp = false;
        m_exitingRamp = true;
        m_position = Math.abs(leftPosition);
      }

      if (m_starting || m_ascending) {
        drive.tankDrive(0.55, 0.55);
      }

      if (m_onFlat || m_descending) {
        drive.tankDrive(0.2, 0.2);
      }

      if (m_exitingRamp){
        if (Math.abs(leftPosition) < m_position + 3){
          drive.tankDrive(0.3, 0.3);
        }
        else {
          m_exitingRamp = false;
          m_startBalancing = true;
          //leftEncoder.setPosition(0);
          m_position = leftPosition - 20;
        }
      }
    
      if (m_startBalancing) {
        if (leftPosition > m_position) {
        //if (Math.abs(vAngle) > 10) {
          drive.tankDrive(-0.6, -0.6);
        } else {
          m_startBalancing = false;
          m_balancing = true;
        }
      }

      if (m_balancing) {
        if (vAngleTest > 2) {
          drive.tankDrive(0.287, 0.287);
        }
        if (vAngleTest < -2) {
          drive.tankDrive(-0.287, -0.287);
        }
      }
        break;
      case kgsdDepositAndBalance:
      if(time - startTime < 5){
        rollerMotor.set(.7);
       }else{
        rollerMotor.set(0);
       }
        enableIntakeMotors(true);
        vAngleTest = vAngleTest * -1;

        if (m_starting && vAngleTest > 5) {
          m_onRamp = true;
          m_ascending = true;
          m_starting = false;
        }

        if (m_ascending && vAngleTest < 0) {
          m_ascending = false;
          m_onFlat = true;
        }

        if (m_onFlat && Math.abs(vAngleTest) > 5) {
          m_onFlat = false;
          m_descending = true;
        }

        if (m_descending && Math.abs(vAngleTest) < 1.5) {
          m_descending = false;
          m_onRamp = false;
          m_exitingRamp = true;
          m_position = Math.abs(leftPosition);
        }

        if (m_starting || m_ascending) {
          drive.tankDrive(0.55, 0.55);
        }

        if (m_onFlat || m_descending) {
          drive.tankDrive(0.2, 0.2);
        }

        if (m_exitingRamp){
          if (Math.abs(leftPosition) < m_position + 4){
            drive.tankDrive(0.3, 0.3);
          }
          else {
            m_exitingRamp = false;
            m_startBalancing = true;
            //leftEncoder.setPosition(0);
            m_position = leftPosition - 20;
          }
        }

        if (m_startBalancing) {
          if (leftPosition > m_position) {
          //if (Math.abs(vAngle) > 10) {
            drive.tankDrive(-0.6, -0.6);
          } else {
            m_startBalancing = false;
            m_balancing = true;
          }
        }

        if (m_balancing) {
          if (vAngleTest > 4) {
            drive.tankDrive(0.278, 0.278);
          }
          if (vAngleTest < -4) {
            drive.tankDrive(-0.278, -0.278);
          }
        }

        break;
      case kDefaultAuto:
        default:
        break;
    }
  }

  @Override
  public void teleopInit() {
    enableDrivingMotors(true);
    enableIntakeMotors(true);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println(Math.round(gyro.getAngle()));
    // drive controls
    double Speed = -driveController.getRawAxis(1); // for this axis: up is negative, down is positive
    double turn = -driveController.getRawAxis(4) * 0.3;
    if(driveController.getBButton()){ // if the B button is pressed the speed is going to be divided in half
      Speed = Speed * 0.9;
      Speed = Speed/2;
      drive.arcadeDrive(Speed, turn);
    }else{
      Speed = -driveController.getRawAxis(1) * 0.9; // speed is reset to 90%
      drive.arcadeDrive(limiter.calculate(Speed), turn); // if the B button is not pressed the "input ramping" is going to be on
    }

    // intake RaisingMotor Control
    double raisingPower = intakeController.getRawAxis(1);
    // deadBand -- just cuz, why not?
    if (Math.abs(raisingPower) < 0.05) {
      raisingPower = 0;
    }

    // going forward
    if (raisingPower < 0 && !frontLimitSensor.get()) {
      raisingPower = 0;
    }

    // going backward
    if (raisingPower > 0 && !backLimitSensor.get()) {
      raisingPower = 0;
    }

    raisingMotor.set(raisingPower * 0.6);
    if ((raisingPower < 0 && frontLimitSensor.get()) || (raisingPower > 0 && backLimitSensor.get())) {
    }else{
      raisingMotor.set(0);
    }

    // intake Rollers control
    double rollersPower = 0;
    // press Y if you want to pick up an object, and press A if you want to shoot
    // the object
    if (intakeController.getYButton() == true) {
      rollersPower = 1;
    } else if (intakeController.getAButton() == true) {
      rollersPower = -0.7;
    }
    // rollersPower = intakeController.getRawAxis(4);
    rollerMotor.set(ControlMode.PercentOutput, rollersPower);
  }

  @Override
  public void disabledInit() {
    enableDrivingMotors(true);
    enableIntakeMotors(true);
  }

  @Override
  public void disabledPeriodic() {
  }

  private void enableDrivingMotors(boolean on) {
    IdleMode dMotormode;
    if (on) {
      dMotormode = IdleMode.kBrake;
    } else {
      dMotormode = IdleMode.kCoast;
    }
    leftFrontMotor.setIdleMode(dMotormode);
    leftBackMotor.setIdleMode(dMotormode);
    rightFrontMotor.setIdleMode(dMotormode);
    rightBackMotor.setIdleMode(dMotormode);
  }

  private void enableIntakeMotors(boolean on) {
    NeutralMode iMotorMode;
    if (on) {
      iMotorMode = NeutralMode.Brake;
    } else {
      iMotorMode = NeutralMode.Coast;
    }

    raisingMotor.setNeutralMode(iMotorMode);
    rollerMotor.setNeutralMode(iMotorMode);
  }
  // --------------------------

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}