package frc.robot.Utilities;

import edu.wpi.first.wpilibj.XboxController;

public class RobotControllerMap {
    private XboxController _driverController;
    private XboxController _operatorController;

    public RobotControllerMap(int driverUsbSlot, int operatorUsbSlot) {
        _driverController = new XboxController(driverUsbSlot);
        _operatorController = new XboxController(operatorUsbSlot);
    }
    public XboxController getDriverController(){
        return _driverController;
    }
    public XboxController getOperatorController(){
        return _operatorController;
    }
}