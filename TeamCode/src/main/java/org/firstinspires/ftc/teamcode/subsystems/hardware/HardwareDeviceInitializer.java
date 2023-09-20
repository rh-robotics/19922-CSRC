package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Defines a callback that is run during a device initialization. */
public interface HardwareDeviceInitializer {
    /** Ditto.
     * @param dev The hardware device to initialize. Must be casted. */
    void initialize(HardwareDevice dev) throws RuntimeException;
}
