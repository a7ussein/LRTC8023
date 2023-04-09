package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.autonomous.AutonomousBase;
import frc.robot.autonomous.DepositAndBalance;
import frc.robot.autonomous.DepositAndDriveForward;
import frc.robot.autonomous.DepositCube;
import frc.robot.autonomous.DoNothing;
import frc.robot.autonomous.OneAndHalf;
import frc.robot.autonomous.TwoCubeAuto;
import frc.robot.autonomous.twoCubeBalance;

public class Robot extends TimedRobot {

    // Auto Selection:
    private static final String kDefaultAuto = "Nothing Auto";
    private static final String kDepositAndDriveForward = "Mobility";
    private static final String kDepositAndBalance = "Deposit & Balance";
    private static final String kDepositCube = "Deposit Cube";
    
    private static final String kOneAndHalf = "One And Half";
    private static final String kTwoCubeAuto = "Two Cube Auto";
    private static final String ktwoCubeBalance = "Two Cube & Balance";
    private static final String kDepositSensor = "Deposit Sensor";
    private static final String kRaiseArm = "Raise Arm";
    // private String m_autoSelected;
    private final SendableChooser<String> auto_chooser = new SendableChooser<>();


    // Driving Motors
    private CANSparkMax leftFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax leftBackMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax rightFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax rightBackMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

    MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    DifferentialDrive drive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

    // Intake Motors
    private WPI_VictorSPX rollerMotor = new WPI_VictorSPX(5);
    private WPI_VictorSPX raisingMotor = new WPI_VictorSPX(6);

    // Encoders
    RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    // Sensors
    // private ADIS16470_IMU gyro = new ADIS16470_IMU();
    // ProximitySensor cubeSensor = new ProximitySensor(7);
    // ProximitySensor frontLimitSensor = new ProximitySensor(9);
    // ProximitySensor backLimitSensor = new ProximitySensor(8);
    

    // Controllers
    private XboxController driveController = new XboxController(Constants.drivingConstants.driveController);
    private XboxController intakeController = new XboxController(Constants.drivingConstants.intakeController);

    // variables
    private final double encoder2inches = 1 / 8.46; // Unit Conversion
    SlewRateLimiter limiter = new SlewRateLimiter(1.0); // Input Ramp

    private Components components = new Components();
    private AutonomousBase autonomous;

    @Override
    public void robotInit() {
        // Auto Selection:
        auto_chooser.setDefaultOption("Nothing Auto", kDefaultAuto);
        auto_chooser.addOption("Deposit Cube", kDepositCube);
        auto_chooser.addOption("Mobility", kDepositAndDriveForward);
        auto_chooser.addOption("Deposit & Balance", kDepositAndBalance);

        auto_chooser.addOption("Two Cube Auto", kTwoCubeAuto);
        auto_chooser.addOption("One And Half", kOneAndHalf);
        auto_chooser.addOption("Two Cube & Balance", ktwoCubeBalance);
        auto_chooser.addOption("DeositSensor", kDepositSensor);
        auto_chooser.addOption("Raise Arm", kRaiseArm);


        SmartDashboard.putData("Auto choices", auto_chooser);

        // Camera init:
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(400, 222);

        // restore mototors factory defaults
        leftFrontMotor.restoreFactoryDefaults();
        leftBackMotor.restoreFactoryDefaults();
        rightFrontMotor.restoreFactoryDefaults();
        rightBackMotor.restoreFactoryDefaults();

        // Invertation Settings
        rightControllerGroup.setInverted(true);
        leftControllerGroup.setInverted(false);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        components.encoder = leftEncoder;
        components.rollerMotor = rollerMotor;
        components.raisingMotor = raisingMotor;
        components.drive = drive;

        components.init();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("frontLimitSenosr", components.frontLimitSensor.get());
        SmartDashboard.putBoolean("backLimitSensor", components.backLimitSensor.get());
        SmartDashboard.putBoolean("CubeSensor", components.cubeSensor.get());
    }

    @Override
    public void autonomousInit() {
        // Auto Stuff:
        switch (auto_chooser.getSelected()) {
            case kDepositAndDriveForward:
                autonomous = new DepositAndDriveForward(components);
            break;
            case kDepositAndBalance:
                autonomous = new DepositAndBalance(components);
            break;
            case kDepositCube:
                autonomous = new DepositCube(components);
            break;
            case kTwoCubeAuto:
                autonomous = new TwoCubeAuto(components);
            break;
            case kOneAndHalf:
                autonomous = new OneAndHalf(components);
            break;
            case ktwoCubeBalance:
                autonomous = new twoCubeBalance(components);
            break;
            case kDefaultAuto:
                autonomous = new DoNothing(components);
            break;
            default:
                autonomous = new DoNothing(components);
            break;
        }

        autonomous.init();
    }

    @Override
    public void autonomousPeriodic() {
        autonomous.periodic();
    }

    @Override
    public void teleopInit() {
        enableDrivingBreak(true);
        enableIntakeBreak(true);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // imput ramping
        rightFrontMotor.setOpenLoopRampRate(0.5);
        rightBackMotor.setOpenLoopRampRate(0.5);
        leftFrontMotor.setOpenLoopRampRate(0.5);
        leftBackMotor.setOpenLoopRampRate(0.5);
    }

    @Override
    public void teleopPeriodic() {
        // drive controls
        double Speed = -driveController.getRawAxis(1) * 0.9; // for this axis: up is negative, down is positive
        double turn = -driveController.getRawAxis(4) * 0.6;
        double vAngleTest = components.gyro.getYComplementaryAngle();
        vAngleTest = vAngleTest * -1; // rio is mounted backwards

        if (driveController.getRightBumper()) { // if the RightBumber is pressed then slow mode is going to be enabled
            drive.arcadeDrive(Speed / 2, turn/2);
        } else if (driveController.getLeftBumper()) { // if both right and left bumbers are pressed then ultra slow mode is going to be enabled
            drive.arcadeDrive(0, 0);
        } else {
            drive.arcadeDrive(Speed, turn); // if the no button is pressed the "input ramping" is going to be on
        }

        // auto balance using the A Button
        if (driveController.getAButton()) {
            if (vAngleTest > 2) {
                drive.tankDrive(0.27, 0.27);
            }
            if (vAngleTest < -2) {
                drive.tankDrive(-0.27, -0.27);
            }
        }



        // intake RaisingMotor Control
        double raisingPower = intakeController.getRawAxis(1);
        // deadBand -- just cuz, why not?
        if (Math.abs(raisingPower) < 0.05) {
            raisingPower = 0;
        }

        // going forward
        if (raisingPower < 0 && components.frontLimitSensor.get()) {
            raisingPower = 0;
        }

        // going backward
        if (raisingPower > 0 && components.backLimitSensor.get()) {
            raisingPower = 0;
        }

        if ((raisingPower < 0 && !components.frontLimitSensor.get()) || (raisingPower > 0 && !components.backLimitSensor.get())) {
            raisingMotor.set(raisingPower * 0.6);
        } else {
            raisingMotor.set(0);
        }

        // intake Rollers control
        double rollersPower = 0;

        /*
         * Operator can only intake using the A button if there is no Cube in the intake
         * Operator can only outtake using the Y button if there is a cube in the intake
         * if something happens to the sensor then the operator can just use the right bumber for intake and the left bumber for outtake 
         */
        if(intakeController.getAButton()) {
            rollersPower = -0.7;
        }else if(intakeController.getYButton()) {
            rollersPower = 1; 
        }
        rollerMotor.set(ControlMode.PercentOutput, rollersPower);
    }

    @Override
    public void disabledInit() {
        enableDrivingBreak(true);
        enableIntakeBreak(true);
    }

    @Override
    public void disabledPeriodic() {
    }

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

    // Functions
    private void enableDrivingBreak(boolean on) {
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
        // armMotor.setIdleMode(dMotormode);
    }

    private void enableIntakeBreak(boolean on) {
        NeutralMode iMotorMode;
        if (on) {
            iMotorMode = NeutralMode.Brake;
        } else {
            iMotorMode = NeutralMode.Coast;
        }

        raisingMotor.setNeutralMode(iMotorMode);
        rollerMotor.setNeutralMode(iMotorMode);
    }
}
