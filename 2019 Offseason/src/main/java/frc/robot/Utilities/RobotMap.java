package frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.*;

public class RobotMap {

    //6 Sparks
    //5 TalonSRXs (2 Arm) (1 Ele) (2 Collector)
    private CANSparkMax _lfDrive;
    private CANSparkMax _lmDrive;
    private CANSparkMax _lrDrive;
    private CANSparkMax _rfDrive;
    private CANSparkMax _rmDrive;
    private CANSparkMax _rrDrive;
    private TalonSRX _leftArm;
    private TalonSRX _rightArm;
    private TalonSRX _elevator;
    private TalonSRX _topCollector;
    private TalonSRX _bottomCollector;
    private Solenoid _collectorSolenoid;
    private DoubleSolenoid _hab3SolenoidLeft;
    private DoubleSolenoid _hab3SolenoidRight;
    private AHRS _gyro;
    private Spark _blinkin;

    public RobotMap(){
        try {
            _lfDrive = new CANSparkMax(Constants.kLFDriveId, MotorType.kBrushless);
            _lfDrive.setSmartCurrentLimit(40);
            _lfDrive.setOpenLoopRampRate(.5);
                        
            _lrDrive = new CANSparkMax(Constants.kLRDriveId, MotorType.kBrushless);
            _lrDrive.setSmartCurrentLimit(40);
            _lrDrive.setOpenLoopRampRate(.5);
            _lrDrive.follow(_lfDrive);

            _rfDrive = new CANSparkMax(Constants.kRFDriveId, MotorType.kBrushless);
            _rfDrive.setSmartCurrentLimit(40);
            _rfDrive.setOpenLoopRampRate(.5);

            _rrDrive = new CANSparkMax(Constants.kRRDriveId, MotorType.kBrushless);
            _rrDrive.setSmartCurrentLimit(40);
            _rrDrive.setOpenLoopRampRate(.5);

            _elevator = new TalonSRX(Constants.kElevatorId);
            _gyro = new AHRS(Port.kMXP);
            _rrDrive.follow(_rfDrive);
        } catch (Exception ex){
            ex.printStackTrace();
        }
        if (!Constants.kIsTestRobot){
            _lmDrive = new CANSparkMax(Constants.kLMDriveId, MotorType.kBrushless);
            _lmDrive.setSmartCurrentLimit(40);
            _lmDrive.setOpenLoopRampRate(.5);
            _lmDrive.follow(_lfDrive);
            
            _rmDrive = new CANSparkMax(Constants.kRMDriveId, MotorType.kBrushless);
            _rrDrive.setSmartCurrentLimit(40);
            _rrDrive.setOpenLoopRampRate(.5);
            _rmDrive.follow(_rfDrive);

            _leftArm = new TalonSRX(Constants.kLArmId);
            _rightArm = new TalonSRX(Constants.kRArmId);
            _topCollector = new TalonSRX(Constants.kTCollectorId);
            _bottomCollector = new TalonSRX(Constants.kBCollectorId);
            _collectorSolenoid = new Solenoid(Constants.kCollectorSolenoidSlot);
            _hab3SolenoidLeft = new DoubleSolenoid(Constants.kHab3SolenoidSlotA, Constants.kHab3SolenoidSlotB);
            _hab3SolenoidRight = new DoubleSolenoid(Constants.kHab3SolenoidSlotC, Constants.kHab3SolenoidSlotD);
            _blinkin = new Spark(Constants.kBlinkinSlot);
        }
        
        System.out.println("Created Robot Map");
        
    }
    public CANSparkMax getLeftDriveSpark(){
        return _lfDrive;
    }
    public CANSparkMax getRightDriveSpark(){
        return _rfDrive;
    }
    public TalonSRX getLeftArmTalon(){
        return _leftArm;
    }
    public TalonSRX getRightArmTalon(){
        return _rightArm;
    }
    public TalonSRX getElevatorTalon(){
        return _elevator;
    }
    public TalonSRX getTopCollectorTalon(){
        return _topCollector;
    }
    public TalonSRX getBottomCollectorTalon(){
        return _bottomCollector;
    }
    public Solenoid getCollectorSolenoid(){
        return _collectorSolenoid;
    }
    public DoubleSolenoid getHab3SolenoidLeft(){
        return _hab3SolenoidLeft;
    }
    public DoubleSolenoid getHab3SolenoidRight(){
        return _hab3SolenoidRight;
    }
    public AHRS getGyro(){
        return _gyro;
    }
    public Spark getBlinkin(){
        return _blinkin;
    }
    public void PrintDiagnostics(){
        //System.out.println("Temp - LF: " + _lfDrive.getMotorTemperature() + " LM: " + _lmDrive.getMotorTemperature() + " LR: " + _lrDrive.getMotorTemperature());
        //System.out.println("Temp - RF: " + _rfDrive.getMotorTemperature() + " RM: " + _rmDrive.getMotorTemperature() + " RR: " + _rrDrive.getMotorTemperature());
        SmartDashboard.putNumber("LeftFrontTemp", _lfDrive.getMotorTemperature());
        //SmartDashboard.putNumber("LeftMidTemp", _lmDrive.getMotorTemperature());
        SmartDashboard.putNumber("LeftRearTemp", _lrDrive.getMotorTemperature());
        SmartDashboard.putNumber("RightFrontTemp", _rfDrive.getMotorTemperature());
        //SmartDashboard.putNumber("RightMidTemp", _rmDrive.getMotorTemperature());
        SmartDashboard.putNumber("RightRearTemp", _rrDrive.getMotorTemperature());
        SmartDashboard.putNumber("LeftFrontCurrent", _lfDrive.getOutputCurrent());
        //SmartDashboard.putNumber("LeftMidCurrent", _lmDrive.getMotorTemperature());
        SmartDashboard.putNumber("LeftRearCurrent", _lrDrive.getOutputCurrent());
        SmartDashboard.putNumber("RightFrontCurrent", _rfDrive.getOutputCurrent());
        //SmartDashboard.putNumber("RightMidCurrent", _rmDrive.getMotorTemperature());
        SmartDashboard.putNumber("RightRearCurrent", _rrDrive.getOutputCurrent());
    }
}