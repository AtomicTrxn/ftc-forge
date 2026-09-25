package com.acmerobotics.dashboard.config;

import java.lang.annotation.*;

/** Marks a class's public static fields as live-tunable via the dashboard. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Config {
    String value() default "";
}
