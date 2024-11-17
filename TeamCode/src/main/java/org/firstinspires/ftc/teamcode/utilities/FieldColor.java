package org.firstinspires.ftc.teamcode.utilities;

public enum FieldColor {
    RED,BLUE;
    
    public static FieldColor active = FieldColor.RED;
    public static FieldColor getActive(){
        return active;
    }
    public static void setField(FieldColor color){
        active = color;
    }
}
