package frc.robot.Configuration;


//Test Robot Constants
/*
public final class Constants {

    //Spark Device ID's
    public static final int kRFDriveId = 2; // Right Front Spark CAN ID
    public static final int kRMDriveId = 3; // Right Middle Spark CAN ID
    public static final int kRRDriveId = 4; // Right Rear Spark CAN ID
    public static final int kLFDriveId = 5; // Left Front Spark CAN ID
    public static final int kLMDriveId = 6; // Left Middle Spark CAN ID
    public static final int kLRDriveId = 7; // Left Rear Spark CAN ID
    public static final int kElevatorId = 8; // Left Elevator Talon CAN ID
    public static final int kRElevatorId = 9; // Right Elevator Talon CAN ID
    public static final int kLArmId = 10; // Left Arm Talon CAN ID
    public static final int kRArmId = 11; // Right Arm Talon CAN ID
    public static final int kTCollectorId = 10; // Top Collector Talon CAN ID
    public static final int kBCollectorId = 11; // Bottom Collector Talon CAN ID
    public static final int kCollectorSolenoidSlot = 2; // Collector Solenoid Slot
    public static final int kHab3SolenoidSlotA = 0; // Hab3 Solenoid Slot A
    public static final int kHab3SolenoidSlotB = 1; // Hab3 Solenoid Slot

    //Controller Usb Slots
    public static final int kDriverControllerSlot = 0;
    public static final int kOperatorControllerSlot = 1;

    //Elevator Setpoints
    public static final double kElevatorNeutral = 0;
    public static final double kElevatorLowHatch = 0;
    public static final double kElevatorLowCargo = 1;
    public static final double kElevatorMidHatch = 2;
    public static final double kElevatorMidCargo = 3;
    public static final double kElevatorTopCargo = 4;

    //Arm Setpoints
    public static final double kArmNeutral = 0;
    public static final double kArmLowHatch = 1;
    public static final double kArmLowCargo = 2;
    public static final double kArmMidHatch = 3;
    public static final double kArmMidCargo = 4;
    public static final double kArmTopCargo = 5;

    //Collector speed
    public static final double kCollectorIntakeSpeed = 0.5;
    public static final double kCollectorReleaseSpeed = -0.5;

    //PID Coefficients
    public static final double kDistancePIDP = 0.01;
    public static final double kDistancePIDI = 0.0001;
    public static final double kDistancePIDD = 0.75;
    public static final double kVelocityPIDP = 0.00025;
    public static final double kVelocityPIDI = 0.0;
    public static final double kVelocityPIDD = 0.0;
    public static final double kVelocityPIDFF = 0.0;

    public static final double kDriveRotationP = 0.021;
    public static final double kDriveRotationI = 0.0000001;
    public static final double kDriveRotationD = 0.0;
    public static final double kDriveRotationTolerance = 0.25; //1 Degree
    
    public static final double kDriveStraightRotationP = 0.0475;
    public static final double kDriveStraightRotationI = 0.0;
    public static final double kDriveStraightRotationD = 0.0;
    
    //Elevator
    public static final double kElevatorP = 0.00025;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorFF = 0.0;

    //Arm
    public static final double kArmP = 0.00025;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;
    public static final double kArmFF = 0.0;

    public static final double kDriveWheelDiameter = 6.0;
    public static final double kDriveVelocityConversion = Math.PI / 107.1; //Rev/Min -> Ft/sec
    public static final double kDrivePositionConversion = 6 * Math.PI / 10.71 / 12; // Ft

    public static final boolean kIsTestRobot = true;

    public static final String kAutonomousPositionLeft = "Left";
    public static final String kAutonomousPositionRight = "Right";
    public static final String kAutonomousPositionCenter = "Center";
    public static final String kAutonomousZoneHab1 = "0";
    public static final String kAutonomousZoneHab2 = "1";
        
}*/
//Competition Robot Constants
public final class Constants {

    //Spark Device ID's
    public static final int kRFDriveId = 2; // Right Front Spark CAN ID
    public static final int kRMDriveId = 3; // Right Middle Spark CAN ID
    public static final int kRRDriveId = 4; // Right Rear Spark CAN ID
    public static final int kLFDriveId = 5; // Left Front Spark CAN ID
    public static final int kLMDriveId = 6; // Left Middle Spark CAN ID
    public static final int kLRDriveId = 7; // Left Rear Spark CAN ID
    public static final int kElevatorId = 8; // Elevator Talon CAN ID
    public static final int kLArmId = 9; // Left Arm Talon CAN ID
    public static final int kRArmId = 10; // Right Arm Talon CAN ID
    public static final int kTCollectorId = 11; // Top Collector Talon CAN ID
    public static final int kBCollectorId = 12; // Bottom Collector Talon CAN ID
    public static final int kCollectorSolenoidSlot = 2; // Collector Solenoid Slot
    public static final int kHab3SolenoidSlotA = 0; // Hab3 Solenoid Slot A
    public static final int kHab3SolenoidSlotB = 1; // Hab3 Solenoid Slot B
    public static final int kHab3SolenoidSlotC = 6;
    public static final int kHab3SolenoidSlotD = 7;
    public static final int kBlinkinSlot = 0;

    //Controller Usb Slots
    public static final int kDriverControllerSlot = 0;
    public static final int kOperatorControllerSlot = 1;

    //Elevator Setpoints
    public static final double kElevatorNeutral = -2600;
    public static final double kElevatorLowHatch = -2600;
    public static final double kElevatorLowCargo = -2600;
    public static final double kElevatorCargoShipCargo = -2600;
    public static final double kElevatorMidHatch = -26800;
    public static final double kElevatorMidCargo = -26800;
    public static final double kElevatorTopCargo = -26800;

    //Arm Setpoints
    public static final double kArmNeutral = 0;
    public static final double kArmGround = -1285;
    public static final double kArmLowHatch = -1050;//1050;
    public static final double kArmCargoShipCargo = -500;
    public static final double kArmLowCargo = -880;
    public static final double kArmMidHatch = -1050;
    public static final double kArmMidCargo = -810;
    public static final double kArmTopCargo = -370;

    //Collector speed
    public static final double kCollectorIntakeSpeed = 0.5;
    public static final double kCollectorReleaseSpeed = -0.35;

    //PID Coefficients
    public static final double kDistancePIDP = 0.01;
    public static final double kDistancePIDI = 0.0001;
    public static final double kDistancePIDD = 0.75;
    public static final double kVelocityPIDP = 0.00025;
    public static final double kVelocityPIDI = 0.0;
    public static final double kVelocityPIDD = 0.0;
    public static final double kVelocityPIDFF = 0.0;

    public static final double kDriveRotationP = 0.021;
    public static final double kDriveRotationI = 0.0000001;
    public static final double kDriveRotationD = 0.0;
    public static final double kDriveRotationTolerance = 0.25; //.5 Degrees
    
    public static final double kDriveStraightRotationP = 0.0475;
    public static final double kDriveStraightRotationI = 0.0;
    public static final double kDriveStraightRotationD = 0.0;
    
    //Elevator
    public static final double kElevatorP = 0.75;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorFF = 0.0;

    //Arm
    public static final double kArmP = .85;
    public static final double kArmI = 0.00025;
    public static final double kArmD = 0.025;
    public static final double kArmFF = 0.0;

    public static final double kDriveWheelDiameter = 6.0;
    public static final double kDriveVelocityConversion = Math.PI / 107.1; //Rev/Min -> Ft/sec
    public static final double kDrivePositionConversion = 6 * Math.PI / 10.71 / 12; // Ft

    public static final boolean kIsTestRobot = false;

    public static final String kAutonomousPositionLeft = "Left";
    public static final String kAutonomousPositionRight = "Right";
    public static final String kAutonomousPositionCenter = "Center";
    public static final String kAutonomousZoneHab1 = "0";
    public static final String kAutonomousZoneHab2 = "1";

    public static final double kBlinkinColorWavesLava = -0.39;
    public static final double kBlinkinColorWavesOcean = -0.41;
    public static final double kBlinkinColorWavesForest = -0.37;
    public static final double kBlinkinSolidRed = 0.61;
    public static final double kBlinkinSolidGreen = 0.77;
    public static final double kBlinkinHeartbeatBlue = -0.23;
    public static final double kBlinkinFireLarge = -0.57;
    
        
}