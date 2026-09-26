package frc.robot.Constants;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public final class CAN {
    public static final class Constants {
        public static final CANBus PigeonCAN = new CANBus(CANPort.CAN_S0);    
        public static final CANPort DriveCAN = CANPort.CAN_S0;
        public static final CANPort ShooterCAN = CANPort.CAN_S1;
    }
}