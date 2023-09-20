package org.firstinspires.ftc.teamcode.subsystems.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * An abstraction over various types of hardware.
 * <p>
 * Using this class, it becomes very easy to make, manage, and use hardware! Thank me.
 */
public class Hardware {
    /**
     * Essentially a more user friendly hardware map, this adds a layer of security.
     */
    private final Map<String, HardwareDevice> registrar;

    /**
     * Moves telemetry away from the opmode and into this handy container.
     */
    private final Telemetry telemetry;

    public Hardware(@NonNull HardwareSchema schema, @NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;

        // Ensure all values in the schema are represented in the hardware map, as well as populate
        // it fully.
        HashMap<String, HardwareDevice> rw_registrar = new HashMap<>();
        schema.inner().forEach((k, v) -> {
            HardwareDevice device = hardwareMap.get(v.getName());
            if (device == null) {
                throw new RuntimeException(v + " ain't present in the active configuration.");
            }

            if (!device.getClass().equals(v.getT())) {
                throw new RuntimeException(v + " ain't of the right type " + v.getT() + ".");
            }

            HardwareDeviceInitializer initializer = v.getInitializer();
            initializer.initialize(device);

            rw_registrar.put(k, device);
        });

        registrar = Collections.unmodifiableMap(rw_registrar);
    }

    /** Get a device loaded into the hardware class. Is casted to the argument supplied.
     * @param name The name of the device to get.
     * @param type The type of the device to cast to.
     */
    public <T> T getDevice(String name, Class<T> type) {
        HardwareDevice device = registrar.get(name);

        if (device != null) {
            return type.cast(device);
        } else {
            // Unreachable because we already checked in the constructor.
            throw new RuntimeException("UNREACHABLE");
        }
    }

    /**
     * Gets the telemetry object.
     */
    public Telemetry getTelemetry() {
        return telemetry;
    }
}