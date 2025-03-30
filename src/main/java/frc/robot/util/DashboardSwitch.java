// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardSwitch implements Sendable {
    boolean active = false;

    public DashboardSwitch() {}

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.setSmartDashboardType("Boolean"); // This sets the type in SmartDashboard
        builder.addBooleanProperty("Active", this::get, this::set);
    }

    /** Returns the current state of the switch. */
    public boolean get() {
        return active;
    }

    /** Sets the state of the switch. */
    public void set(boolean value) {
        this.active = value;
    }
}
