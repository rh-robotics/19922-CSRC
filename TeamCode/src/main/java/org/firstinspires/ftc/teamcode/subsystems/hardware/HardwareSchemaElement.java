package org.firstinspires.ftc.teamcode.subsystems.hardware;

/** An element of the hardware schema. Contains the name in the hardware map and an initialization
 * callback for settings up a device once it's gotten and before it's stored. */
public class HardwareSchemaElement<T> {
    /** The name of the schema element. */
    private final String name;

    /** The initializer callback. */
    private final HardwareDeviceInitializer initializer;

    /** The type of the device. */
    private final java.lang.Class<? extends T> t;

    /** Constructs a HardwareSchemaElement.
     * @param t The type of the device.
     * @param name The name of the device.
     * @param initializer The initializer callback. */
    public HardwareSchemaElement(java.lang.Class<? extends T> t, String name,
                                 HardwareDeviceInitializer initializer) {
        this.t = t;
        this.name = name;
        this.initializer = initializer;
    }

    /** Get the type of the device.
     * @return Ditto */
    public Class<? extends T> getT() {
        return t;
    }

    /** Get the name of the device.
     * @return Ditto. */
    public String getName() {
        return name;
    }

    /** Get the initializer callback.
     * @return Ditto. */
    public HardwareDeviceInitializer getInitializer() {
        return initializer;
    }
}
