package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.OpModeProvider;

public class Arm extends BasicComponent {
    private static final int EXTENSION_MARGIN = 5;
    private boolean isAtTarget = true;
    private int extensionTarget = 0;
    private int extensionCurrent = 0;
    private DcMotor armMotorLeft, armMotorRight;

    public Arm(@NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "Arm");
    }

    @Override
    public void init() {
        assert(hardwareMap != null);

        armMotorLeft = hardwareMap.get(DcMotor.class, "pulleyL");
        armMotorRight = hardwareMap.get(DcMotor.class, "pulleyR");
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        assert(armMotorLeft != null);
        assert(armMotorRight != null);
    }

    @Override
    public void loop() {
        armMotorLeft.setTargetPosition(extensionTarget);
        armMotorRight.setTargetPosition(extensionTarget);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotorLeft.setPower(1.0);
        armMotorRight.setPower(1.0);

        extensionCurrent = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;
        isAtTarget = Math.abs(extensionTarget - extensionCurrent) < EXTENSION_MARGIN;

        telemetry.addData("Target Reached (Arm)", isAtTarget);
        telemetry.addData("Extension Target", extensionTarget);
        telemetry.addData("Extension Current", extensionCurrent);
        telemetry.addData("Motors Are Busy", armMotorLeft.isBusy() + ", " + armMotorRight.isBusy());
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
