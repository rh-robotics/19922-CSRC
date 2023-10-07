package org.firstinspires.ftc.teamcode.subsystems.fsm;

import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;

/**
 * Describes some metadata about a state.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface StateMeta {
    /** The color to use when graphing with GraphViz DOT. */
    String color() default "#ffffff";

    /** How to use the state. */
    StateUsage usage() default StateUsage.TRANSITIONAL;
}
