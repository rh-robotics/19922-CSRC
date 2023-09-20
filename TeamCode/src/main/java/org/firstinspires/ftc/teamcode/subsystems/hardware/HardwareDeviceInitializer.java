package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Defines a callback that is run during a device initialization. */
public interface HardwareDeviceInitializer {
    void initialize(HardwareDevice dev) throws RuntimeException;
}
