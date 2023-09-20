package org.firstinspires.ftc.teamcode.subsystems.hardware;

/** An element of the hardware schema. Contains the name in the hardware map and an initialization
 * callback for settings up a device once it's gotten and before it's stored. */
public class HardwareSchemaElement<T> {
    private final String name;
    private final HardwareDeviceInitializer initializer;
    private final java.lang.Class<? extends T> t;

    public HardwareSchemaElement(java.lang.Class<? extends T> t, String name,
                                 HardwareDeviceInitializer initializer) {
        this.t = t;
        this.name = name;
        this.initializer = initializer;
    }

    public Class<? extends T> getT() {
        return t;
    }

    public String getName() {
        return name;
    }

    public HardwareDeviceInitializer getInitializer() {
        return initializer;
    }
}
