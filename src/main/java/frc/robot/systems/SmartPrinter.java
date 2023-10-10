// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.security.InvalidParameterException;
import java.util.ArrayList;

import frc.robot.SmartPrintable;

public class SmartPrinter {
    private ArrayList<SmartPrintable> printables;
    private static final SmartPrinter instance = new SmartPrinter();

    public SmartPrinter() {
        printables = new ArrayList<SmartPrintable>();
    }

    public static void register(SmartPrintable printable) {
        if (printable == null) {
            throw new InvalidParameterException("Cannot pass null to SmartPrinter.register()");
        }

        instance.printables.add(printable);
    }

    public static void print() {
        for (SmartPrintable printable : instance.printables) {
            printable.print();
        }
    }
}
