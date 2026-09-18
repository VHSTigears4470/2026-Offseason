package frc.robot.Constants;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public final class CAN {
    public static final class Constants {
        public static final CANBus PigeonCAN = CANBus.systemcore(0);    
        public static final CANPort DriveCAN = CANPort.CAN_D0;
        public static final CANPort ShooterCAN = CANPort.CAN_D1;

    }
}
