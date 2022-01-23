package frc.robot;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.ExternalLib.NorthwoodLib.Telemety.CustomLayout;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
import frc.robot.RobotContainer;

public class ShowInputTelemtry {
    private final ShuffleboardTab tab;
    private ControllerLayout driver;
    

 

    public ShowInputTelemtry(){
        tab = Shuffleboard.getTab("Input");


    }
    public void initialize(){
        driver = new ControllerLayout("Driver", tab, RobotContainer.m_controller );

    }





    public class ControllerLayout extends CustomLayout{
        public NetworkTableEntry LeftX;
        public NetworkTableEntry LeftY;
        public NetworkTableEntry RightX;
        public NetworkTableEntry RightY;

        private SpectrumXboxController controller;





        
        public ControllerLayout(String name, ShuffleboardTab chosenTab, SpectrumXboxController controller){
            super(name, chosenTab);
           
            this.controller = controller;
            setColumnsAndRows(2, 6);
            setSize(2, 4);
        

            

        }
        public void initialize(){
            LeftX = quickAddPersistentWidget("StickLeft X Command", controller.leftStick.getX(), 0, 0);
            LeftY = quickAddPersistentWidget("StickLeft Y Command", controller.leftStick.getY(), 1, 0);

            RightX = quickAddPersistentWidget("StickRight X", controller.rightStick.getX(), 0, 1);
            RightY = quickAddPersistentWidget("StickRight Y", controller.rightStick.getY(), 1, 1);


        }
        public void update(){
            
        }





    }
    
}
