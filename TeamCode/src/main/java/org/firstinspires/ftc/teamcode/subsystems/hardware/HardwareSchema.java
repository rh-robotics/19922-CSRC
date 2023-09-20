package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashMap;

/** Allows for early-runtime safety by ensuring all expected devices are connected correctly.
 * Note that this class does not do the checking; that is done by the hardware class.
 */
public class HardwareSchema {
    /** The key is the key that can be accessed from the Hardware class, and the value is the key
     * that is expected to be present in the hardware map (FTC). */
    private HashMap<String, HardwareSchemaElement<?>> schema;

    /**
     * Create an entry in the schema.
     * @param key The key that can be accessed from the Hardware class.
     * @param entry The value that is expected to be in the hardware map.
     * @param t The class of the device to introduce.
     * @param initializer The callback used to initialize the device.
     * @param <T> The class of device to introduce.
     */
    public <T> void introduce(String key, String entry, java.lang.Class<? extends HardwareDevice> t,
                              HardwareDeviceInitializer initializer) {
        this.schema.put(key, new HardwareSchemaElement<>(t, entry, initializer));
    }

    /**
     * Destroy an entry in the schema.
     * @param key The key to destroy.
     */
    public void destroy(String key) {
        this.schema.remove(key);
    }

    /** Returns in the inner schema map. Should only be used by the Hardware class.
     * @return The inner schema representation. */
    public HashMap<String, HardwareSchemaElement<?>> inner() {
        return this.schema;
    }
}
