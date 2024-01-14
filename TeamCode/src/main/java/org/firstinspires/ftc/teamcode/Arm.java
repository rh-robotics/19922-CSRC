package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.OpModeProvider;

public class Arm extends BasicComponent {
    private static final int EXTENSION_MARGIN = 5;
    private boolean isAtTarget = true;
    private int extensionTarget = 0;
    private int extensionCurrent = 0;
    private DcMotor armMotor;

    public Arm(@NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "Arm");
    }

    @Override
    public void start() {
        assert(hardwareMap != null);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
    }

    @Override
    public void loop() {
        armMotor.setTargetPosition(extensionTarget);
        extensionCurrent = armMotor.getCurrentPosition();
        isAtTarget = Math.abs(extensionTarget - extensionCurrent) < EXTENSION_MARGIN;

        telemetry.addData("Target Reached (Arm)", isAtTarget);
    }

    @Override
    public void stop() {
        retract();
    }

    public void retract() {
        setExtensionTarget(0);
    }

    public void setExtensionTarget(int extensionTarget) {
        this.extensionTarget = extensionTarget;
    }

    public boolean isAtTarget() {
        return isAtTarget;
    }
}
