package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.ProfilePoint;
import frc.robot.Utilities.TrapezoidalMotionProfile;

public class DriveTrain implements Subsystem {
    private DifferentialDrive _robotDrive;
    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;
    protected AHRS _gyro;
    protected CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;
    private CANPIDController _leftSparkPID;
    private CANPIDController _rightSparkPID;
    private PIDController _rotationPID;

    private boolean _pidEnabled;

    private NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry _angleEntry = _table.getEntry("tx");

    public DriveTrain(CANSparkMax leftMotor, CANSparkMax rightMotor, AHRS gyro){
        try {
            _robotDrive = new DifferentialDrive(leftMotor, rightMotor);
            _leftSpark = leftMotor;
            _rightSpark = rightMotor;
            _leftSpark.setControlFramePeriodMs(10);
            _rightSpark.setControlFramePeriodMs(10);
            _leftSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            _rightSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            _gyro = gyro;
        } catch (Exception ex){
            ex.printStackTrace();
        }

        try {
            _leftEncoder = _leftSpark.getEncoder();
            _leftEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            _leftEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _rightEncoder = _rightSpark.getEncoder();
            _rightEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            _rightEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _leftSparkPID = _leftSpark.getPIDController();
            _rightSparkPID = _rightSpark.getPIDController();
        } catch (Exception ex){
            ex.printStackTrace();
        }
    }
    public void setRealWorldUnits(){
        _leftEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
        _leftEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

        _rightEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
        _rightEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);
    }
    public void updatePIDFromDashboard() {
        _leftSparkPID.setP(SmartDashboard.getNumber("leftSparkP", Constants.kVelocityPIDP));
        _leftSparkPID.setI(SmartDashboard.getNumber("leftSparkI", Constants.kVelocityPIDI));
        _leftSparkPID.setD(SmartDashboard.getNumber("leftSparkD", Constants.kVelocityPIDD));
        _leftSparkPID.setFF(SmartDashboard.getNumber("leftSparkFF", Constants.kVelocityPIDFF));

        _rightSparkPID.setP(SmartDashboard.getNumber("rightSparkP", Constants.kVelocityPIDP));
        _rightSparkPID.setI(SmartDashboard.getNumber("rightSparkI", Constants.kVelocityPIDI));
        _rightSparkPID.setD(SmartDashboard.getNumber("rightSparkD", Constants.kVelocityPIDD));
        _rightSparkPID.setFF(SmartDashboard.getNumber("rightSparkFF", Constants.kVelocityPIDFF));
        
        _rotationPID.setP(SmartDashboard.getNumber("rotationPIDP", Constants.kDriveRotationP));
        _rotationPID.setI(SmartDashboard.getNumber("rotationPIDI", Constants.kDriveRotationI));        
        _rotationPID.setD(SmartDashboard.getNumber("rotationPIDD", Constants.kDriveRotationD));
    }
    public void runDriveMotors(double leftSpeed, double rightSpeed){
        _leftSparkPID.setReference(leftSpeed, ControlType.kVoltage);
        _rightSparkPID.setReference(rightSpeed, ControlType.kVoltage);
    }
    public void stop() {
        _leftSpark.set(0.0);
        _rightSpark.set(0.0);
    }
    public void ensureBrakeMode(){
        if (_leftSpark.getIdleMode() == IdleMode.kCoast || _rightSpark.getIdleMode() == IdleMode.kCoast){
            _leftSpark.setIdleMode(IdleMode.kBrake);
            _rightSpark.setIdleMode(IdleMode.kBrake);
        }
    }public void ensureCoastMode(){
        if (_leftSpark.getIdleMode() == IdleMode.kBrake || _rightSpark.getIdleMode() == IdleMode.kBrake){
            _leftSpark.setIdleMode(IdleMode.kCoast);
            _rightSpark.setIdleMode(IdleMode.kCoast);
        }
    }
    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        ensureCoastMode();
        double dampen = 0.90;
        double throttle = -speed * dampen;
        _robotDrive.curvatureDrive(throttle, rotation, isQuickTurn);
    }
    public void arcadeDrive(double speed, double rotation){
        ensureCoastMode();
        double dampen = 0.90;
        double throttle = -speed * dampen;
        _robotDrive.arcadeDrive(throttle, rotation, true);
    }
    public void tankDrive(double leftSpeed, double rightSpeed){
        _robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
    public void WriteToDashboard(){
        if (Constants.kIsTestRobot){
            updatePIDFromDashboard();
        }
        SmartDashboard.putNumber("leftDrivePosition", _leftEncoder.getPosition());
        SmartDashboard.putNumber("leftDriveVelocity", _leftEncoder.getVelocity());
        SmartDashboard.putNumber("rightDrivePosition", _rightEncoder.getPosition());
        SmartDashboard.putNumber("rightDriveVelocity", _rightEncoder.getVelocity());
        SmartDashboard.putNumber("rightDriveOutput", _rightSpark.get());
        SmartDashboard.putNumber("leftDriveOutput", _leftSpark.get());
        SmartDashboard.putNumber("gyroAngle", _gyro.getAngle());
        SmartDashboard.putNumber("rotationPIDSetpoint", _rotationPID.getSetpoint());
        SmartDashboard.putNumber("rotationPIDP", _rotationPID.getP());
        SmartDashboard.putNumber("rotationPIDI", _rotationPID.getI());
        SmartDashboard.putNumber("rotationPIDD", _rotationPID.getD());
        SmartDashboard.putNumber("rotationPIDOutput", _rotationPID.get());
        
    }

    @Override
    public void ResetSensors() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
        _gyro.reset();
    }
    public void resetEncoders(){
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }
    public double getLeftDistance(){
        return _leftEncoder.getPosition();
    }
    public double getRightDistance(){
        return _rightEncoder.getPosition();
    }
    public double getAngle(){
        return _gyro.getYaw();
    }
    public boolean isPIDEnabled()
    {
        return _pidEnabled;
    }


}