package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;

public class Components {
    public WPI_VictorSPX rollerMotor;
    public RelativeEncoder encoder;
    public ADIS16470_IMU gyro = new ADIS16470_IMU();
    public DifferentialDrive drive;

    public static final String kLow = "Low";
    public static final String kMid = "Mid";
    public final SendableChooser<String> targetChooser = new SendableChooser<>();


    public void init() {
        targetChooser.addOption("Low", kLow);
        targetChooser.addOption("Middle", kMid);
        SmartDashboard.putData("Target Choices", targetChooser);

        gyro.calibrate();
        gyro.reset();
    }
}