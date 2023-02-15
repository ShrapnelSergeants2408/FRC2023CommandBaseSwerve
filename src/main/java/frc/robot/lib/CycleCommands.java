// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class CycleCommands {

    private final CommandBase[] commands;
    private final String label;
    private int activeCommand=0;
    public CycleCommands(String label, CommandBase[] commands, JoystickButton previous, JoystickButton next){
        this.label=label;
        this.commands = commands;
        if (previous.getAsBoolean()) {
            this.previousCommand();
        }
        //previous.getAsBoolean()(this.previousCommand());
        if (next.getAsBoolean()){
            this.nextCommand();
        }
        //next.onTrue(this::nextCommand());
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }

    private void nextCommand(){
        activeCommand += 1;
        if (activeCommand >= commands.length){
            activeCommand=commands.length;
        }
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }

    private void previousCommand(){
        activeCommand -= 1;
        if (activeCommand <0 ){
            activeCommand = 0;
        }
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }
}