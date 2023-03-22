package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;

public class Components {
    public WPI_VictorSPX rollerMotor;
    public RelativeEncoder encoder;
    public ADIS16470_IMU gyro = new ADIS16470_IMU();
    public DifferentialDrive drive;

    public void init() {
        gyro.calibrate();
        gyro.reset();
    }
}