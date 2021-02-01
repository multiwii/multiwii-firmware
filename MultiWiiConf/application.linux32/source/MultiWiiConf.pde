import processing.serial.Serial; // serial library
import controlP5.*; // controlP5 library
import processing.opengl.*; 
import java.lang.StringBuffer; // for efficient String concatemation
import javax.swing.SwingUtilities; // required for swing and EDT
import javax.swing.JFileChooser; // Saving dialogue
import javax.swing.filechooser.FileFilter; // for our configuration file filter "*.mwi"
import javax.swing.JOptionPane; // for message dialogue

//Added For  Processing 2.0.x compabillity
import java.util.*;
import java.io.*;
//****************************

// TODO add new msp :  pid description with bound and scale

PrintWriter output;
BufferedReader reader;
String portnameFile="SerialPort.txt"; // Name of file for Autoconnect.
int GUI_BaudRate = 115200; // Default.
int SerialPort;
Serial g_serial;
ControlP5 controlP5;
Textlabel txtlblWhichcom,TxtInfoMotors1;
Textlabel txtlblRates,txtlblRev,TxtLeftW,TxtRightW,TxtRevW,TxtRevR,TxtRates,TxtRev,
          TxtMids,TxtMin,TxtMax,TxtSLeft,TxtSNick,TxtSRight,TxtInfo,TxtInfo1,TxtInfo2,
          TxtInfo3,TxtInfo4,TxtAux,Links;

ListBox commListbox,baudListbox;

static int CHECKBOXITEMS=0;
static int PIDITEMS=10;
int commListMax;
int tabHeight=20; // Extra height needed for Tabs
int Centerlimits[]  = {1200,1800}; // Endpoints of ServoCenterSliders 

cGraph g_graph;
int windowsX    = 1000;       int windowsY    = 550+tabHeight;
int xGraph      = 10;         int yGraph      = 325+tabHeight;
int xObj        = 520;        int yObj        = 293+tabHeight;
int xCompass    = 920;        int yCompass    = 341+tabHeight;
int xLevelObj   = 920;        int yLevelObj   = 80+tabHeight; 
int xParam      = 120;        int yParam      = 5+tabHeight;
int xRC         = 690;        int yRC         = 10+tabHeight; 
int xMot        = 690;        int yMot        = 155+tabHeight;
int xButton     = 845;        int yButton     = 231+tabHeight;
int xBox        = 415;        int yBox        = 10+tabHeight;
int xGPS        = 853;        int yGPS        = 438+tabHeight;
int xServ       = 350;        int yServ       = 20+tabHeight; 

int i, j; // enummerators
int byteRC_RATE,byteRC_EXPO, byteRollPitchRate,byteYawRate,
    byteDynThrPID,byteThrottle_EXPO, byteThrottle_MID, byteSelectSetting,
    cycleTime, i2cError,
    version, versionMisMatch,horizonInstrSize,
    GPS_distanceToHome, GPS_directionToHome,
    GPS_numSat, GPS_fix, GPS_update, GPS_altitude, GPS_speed,
    GPS_latitude, GPS_longitude,
    init_com, graph_on, pMeterSum, intPowerTrigger, bytevbat, amperage, rssi ;
    
int multiCapability = 0; // Bitflags stating what capabilities are/are not present in the compiled code.
int byteMP[] = new int[8];  // Motor Pins.  Varies by multiType and Arduino model (pro Mini, Mega, etc).
int MConf[]  = new int[10]; // Min/Maxthro etc
int byteP[] = new int[PIDITEMS], byteI[] = new int[PIDITEMS], byteD[] = new int[PIDITEMS];
int activation[];
int ServoMID[]       = new int[8];  // Plane,ppm/pwm conv,heli
int servoRATE[]      = new int[8];
int servoDirection[] = new int[8];
int ServoMIN[]       = new int[8];
int ServoMAX[]       = new int[8];
int wingDir[]        = new int[8];  // Flying wing
int wingPos[]        = new int[8];
int In[]             = new int[8];

Textlabel TxtMIX[]   = new Textlabel[6];

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...
// Alias for multiTypes
int TRI           =1;
int QUADP         =2;
int QUADX         =3;
int BI            =4;
int GIMBAL        =5;
int Y6            =6;
int HEX6          =7;
int FLYING_WING   =8;
int Y4            =9;
int HEX6X         =10;
int OCTOX8        =11;
int OCTOFLATX     =12;
int OCTOFLATP     =13;
int AIRPLANE      =14;
int HELI_120_CCPM =15;
int HELI_90_DEG   =16;
int VTAIL4        =17;
int HEX6H         =18;
int PPM_TO_SERVO  =19;
int DUALCOPTER    =20;
int SINGLECOPTER  =21;


float gx, gy, gz, ax, ay, az, magx, magy, magz, alt, head, angx, angy,
      debug1, debug2, debug3, debug4,
      angyLevelControl, angCalc,
      pVersion;

float mot[] = new float[8],
      servo[] = new float[8],
      RCChan[] = new float[16];

private static final int ROLL = 0, PITCH = 1, YAW = 2, ALT = 3, VEL = 4, LEVEL = 5, MAG = 6;

boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,altGraph=true,headGraph=true, magxGraph =true,magyGraph=true,magzGraph=true,
        debug1Graph = false,debug2Graph = false,debug3Graph = false,debug4Graph = false,hideDraw=false,GraphicsInited=false,gimbalConfig=false,flapperons=false,
        flaps=false,InitServos=true;

boolean toggleServo=false,toggleWriteServo=false,toggleWing=false,toggleWriteWing=false,toggleLive=false,toggleWriteServoLive=false,toggleWriteWingLive=false,
        toggleSaveHeli=false,toggleWaitHeli=false,toggleGimbal=false,
	graphEnabled = false,Mag_=false,gimbal=false, servoStretch=false,camTrigger=false,ExportServo=false,
        toggleTrigger=false,ServosActive=false;

static int RCThro = 3, RCRoll = 0, RCPitch =1, RCYaw =2, RCAUX1=4, RCAUX2=5, RCAUX3=6, RCAUX4=7;

cDataArray accPITCH   = new cDataArray(200), accROLL    = new cDataArray(200), accYAW     = new cDataArray(200),
           gyroPITCH  = new cDataArray(200), gyroROLL   = new cDataArray(200), gyroYAW    = new cDataArray(200),
           magxData   = new cDataArray(200), magyData   = new cDataArray(200), magzData   = new cDataArray(200),
           altData    = new cDataArray(200), headData   = new cDataArray(200),
           debug1Data = new cDataArray(200), debug2Data = new cDataArray(200), debug3Data = new cDataArray(200),
           debug4Data = new cDataArray(200);


Numberbox confP[]   = new Numberbox[PIDITEMS],
          confI[]   = new Numberbox[PIDITEMS],
          confD[]   = new Numberbox[PIDITEMS],
          confINF[] = new Numberbox[7],
          VBat[]    = new Numberbox[6]
          ;

Numberbox confRC_RATE, confRC_EXPO, rollPitchRate, yawRate, dynamic_THR_PID, throttle_EXPO, throttle_MID, 
          confPowerTrigger, confSetting, confSelectSetting;


Slider axSlider, aySlider, azSlider, gxSlider, gySlider, gzSlider, magxSlider, magySlider,
       magzSlider, altSlider, headSlider, debug1Slider, debug2Slider, debug3Slider, debug4Slider,scaleSlider,motorControlSlider;

Slider servoSliderH[]     = new Slider[8],
       servoSliderV[]     = new Slider[8],
       motSlider[]        = new Slider[8],
       TX_StickSlider[]   = new Slider[8],
       GimbalSlider[]     = new Slider[12],
       ServoSliderC[]     = new Slider[8],
       ServoSliderMAX[]   = new Slider[8],
       ServoSliderMIN[]   = new Slider[8];


Button buttonIMPORT, buttonSAVE, buttonREAD, buttonRESET, buttonWRITE, buttonCALIBRATE_ACC, buttonCALIBRATE_MAG, buttonSTART, buttonSTOP, buttonSETTING, 
       buttonAcc, buttonBaro, buttonMag, buttonGPS, buttonSonar, buttonOptic, buttonRXbind, btnQConnect,buttonExport,
       btMagDecl,btMWiiHome,btDownloads;

Button SaveSERVO,buttonSERVO,buttonWing,SaveWing,buttonLIVE,buttonCCPM,buttonGimbal,btnTrigger;

Toggle tACC_ROLL, tACC_PITCH, tACC_Z, tGYRO_ROLL, tGYRO_PITCH, tGYRO_YAW, tBARO,tHEAD, tMAGX, tMAGY, tMAGZ, 
        tDEBUG1, tDEBUG2, tDEBUG3, tDEBUG4;

// Motors Toggles
Toggle motToggle[] = new Toggle[8];

// Colors
color yellow_ = color(200, 200, 20), green_ = color(30, 120, 30), red_ = color(120, 30, 30), blue_ = color(50, 50, 100),
       grey_ = color(30, 30, 30),black_ = color(0, 0, 0),orange_ =color(200,128,0);

PFont font8, font9, font12, font15;

CheckBox checkbox[], checkboxRev[];
CheckBox Bbox,  Wbox,  Mbox;
Button buttonCheckbox[], BtServo[], BtAUX[];
Numberbox RateSlider[];

// TODO New part
void create_GimbalGraphics(){
  if(gimbal){
  gimbalConfig = true;
  int sMin=1020;int sMax=2000;
  if(servoStretch) {sMin=500; sMax=2500;}
  int Step=yServ-10; 
  GimbalSlider[0] = controlP5.addSlider("Tilt_Min" ,sMin,1500,0,xServ+10,Step+80,60,10)  .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[1] = controlP5.addSlider("Tilt_Max" ,1500,sMax,0,xServ+150 ,Step+80,60,10).setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[2] = controlP5.addSlider("Channel " ,1200,1700,0,xServ+100,Step+60,90,10) .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[3] = controlP5.addSlider("Tilt_Prop",-125,125,0,xServ+100,Step+100, 60,10)  .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  Step+=90;
  GimbalSlider[4] = controlP5.addSlider("Roll_Min" ,sMin,1500,0,xServ+10,Step+80,60,10)  .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[5] = controlP5.addSlider("Roll_Max" ,1500,sMax,0,xServ+150 ,Step+80,60,10).setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[6] = controlP5.addSlider("Channel" ,1200,1700,0,xServ+100,Step+60,90,10)  .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[7] = controlP5.addSlider("Roll_Prop",-125,125,0,xServ+100,Step+100, 60,10)  .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  
  GimbalSlider[8] = controlP5.addSlider("Trig_LO" ,500,2000,0,xServ+10,Step+80,60,10)   .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[9] = controlP5.addSlider("Trig_HI" ,1000,sMax,0,xServ+150 ,Step+80,60,10) .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[10] = controlP5.addSlider("Trigger" ,0,30000,0,xServ+100,Step+60,90,10)   .setDecimalPrecision(0).hide().moveTo("ServoSettings");
  GimbalSlider[11] = controlP5.addSlider("Trig_Rev",-0,1,0,xServ+100,Step+100, 40,10)    .setDecimalPrecision(0).hide().moveTo("ServoSettings") .setNumberOfTickMarks(2).setColorTickMark(0);

  buttonGimbal.show();
  if(camTrigger)btnTrigger.show();
  controlP5.getTab("ServoSettings").show();
  }
}
  
void create_ServoGraphics(){
  BtServo = new Button[8];
  BtAUX   = new Button[5];
  checkboxRev = new CheckBox[8];
  RateSlider = new Numberbox[8];
  GraphicsInited=true;
  Bbox = controlP5.addCheckBox("Bbox").setPosition(xServ+40,yServ+40).setColorForeground(color(120))
                .setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10).setColorBackground(color(140))
                .setItemsPerRow(1).setSpacingColumn(30).setSpacingRow(10)
                .addItem("0", 254).addItem("1", 254).addItem("2", 254)
                .addItem("3", 254).addItem("4", 254).addItem("5", 254)
                .addItem("6", 254).addItem("7", 254)
                .hide().hideLabels().moveTo("ServoSettings")
                ;
 
 //Create Common ServoSliders
 int Step =0; 
  int sMin=1020;int sMax=2000;
  if(servoStretch) {sMin=500; sMax=2500;}
   for (i=0;i<8;i++) {
     ServoSliderC[i]    =  controlP5.addSlider("Servo "+i,Centerlimits[0],Centerlimits[1],0,xServ+180,yServ+40+Step,110,12).setDecimalPrecision(0).hide().setLabel("").moveTo("ServoSettings");
     ServoSliderMIN[i]  =  controlP5.addSlider("MIN "+i,sMin,1500,0,xServ+80,yServ+40+Step,40,12).setDecimalPrecision(0).setLabel("").hide().moveTo("ServoSettings");
     ServoSliderMAX[i]  =  controlP5.addSlider("MAX "+i,1500,sMax,0,xServ+125,yServ+40+Step,40,12).setDecimalPrecision(0).setLabel("").hide().moveTo("ServoSettings");
     Step+=20; }
	 
 // ServoGraphics For AirPlane SC & DC
if(multiType == PPM_TO_SERVO || multiType == AIRPLANE ){
  controlP5.getTab("ServoSettings").show();
   TxtRates  = controlP5.addTextlabel("label","Servo Rates in %").setPosition(xServ+75,yServ+20).hide().moveTo("ServoSettings");
   TxtMids   = controlP5.addTextlabel("Mlabel","Offset for servos").setPosition(xServ+190,yServ+20).hide().moveTo("ServoSettings");
   TxtAux   = controlP5.addTextlabel("Alabel","Channel for Flaps").setPosition(xServ-120,yServ+20).hide().moveTo("ServoSettings");
   TxtRev    = controlP5.addTextlabel("txtlblRev","Norm/Rev").setPosition(xServ+10,yServ+20).hide().moveTo("ServoSettings");
 
//****************** End of AirPlane ********************
}

if(multiType == FLYING_WING || multiType == TRI || multiType == BI  || multiType == DUALCOPTER || multiType == SINGLECOPTER){ //ServoGraphics For FlyingWng & TRI & BI || multiType == DUALCOPTER
controlP5.getTab("ServoSettings").show();
int LabelPos=100;
 
if(multiType == FLYING_WING){
  Wbox = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(2).setSpacingColumn(50).setSpacingRow(10).setPosition(xServ-160  ,yServ+50)
       .addItem("L Roll", 254).addItem("R Roll", 254).addItem("L NICK", 254).addItem("R NICK", 254).moveTo("ServoSettings") ;
   TxtLeftW  = controlP5.addTextlabel("Label","Left Wing"  ).setPosition(xServ+110,yServ+40).hide().moveTo("ServoSettings");
   TxtRightW = controlP5.addTextlabel("Rlabel","Right Wing").setPosition(xServ+110,yServ+130).hide().moveTo("ServoSettings");
 }
 if(multiType == TRI){
   Wbox = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(1).setSpacingColumn(50).setSpacingRow(10).setPosition(xServ-160  ,yServ+50)
       .addItem("YAW", 254).moveTo("ServoSettings") ;
    TxtLeftW  = controlP5.addTextlabel("Label","Yaw Servo"  ).setPosition(xServ+110,yServ+40).hide().moveTo("ServoSettings");
    TxtRightW = controlP5.addTextlabel("Rlabel"," ").setPosition(xServ+110,yServ+140).hide().moveTo("ServoSettings");    
  }
  if(multiType == BI){
  Wbox = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(2).setSpacingColumn(50).setSpacingRow(10).setPosition(xServ-160  ,yServ+50)
       .addItem("L Yaw", 254).addItem("R Yaw", 254).addItem("L NICK", 254).addItem("R NICK", 254).moveTo("ServoSettings") ;
   TxtLeftW  = controlP5.addTextlabel("Label","Left Servo"  ).setPosition(xServ+110,yServ+40).hide().moveTo("ServoSettings");
   TxtRightW = controlP5.addTextlabel("Rlabel","Right Servo").setPosition(xServ+110,yServ+140).hide().moveTo("ServoSettings");
 }
 if(multiType == DUALCOPTER){
  Wbox = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(2).setSpacingColumn(70).setSpacingRow(10).setPosition(xServ-160  ,yServ+50)
       .addItem("Pitch ", 254).addItem("Roll ", 254).moveTo("ServoSettings").hide();
   TxtLeftW  = controlP5.addTextlabel("Label","Roll"  ).setPosition(xServ+110,yServ+40).hide().moveTo("ServoSettings");
   TxtRightW = controlP5.addTextlabel("Rlabel","Nick").setPosition(xServ+110,yServ+130).hide().moveTo("ServoSettings");
 }
 if(multiType == SINGLECOPTER){
  Wbox = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(2).setSpacingColumn(50).setSpacingRow(10).setPosition(xServ-80  ,yServ+100)
       .addItem(" Right ", 254).addItem("R yaw", 254)
       .addItem(" Left ", 254) .addItem("L yaw", 254)
       .addItem(" Front", 254) .addItem("F yaw", 254)
       .addItem(" Rear", 254)  .addItem("  yaw", 254)
       .moveTo("ServoSettings") ;
LabelPos=60;
 TxtMin    = controlP5.addTextlabel("Minlabel","MIN").setPosition(xServ+85,yServ+80) .hide().moveTo("ServoSettings");
 TxtMax    = controlP5.addTextlabel("Maxlabel","MAX").setPosition(xServ+130,yServ+80).hide().moveTo("ServoSettings");
 TxtMids   = controlP5.addTextlabel("Mlabel","Offset servos").setPosition(xServ+190,yServ+80).hide().moveTo("ServoSettings");
 }

   TxtRevW   = controlP5.addTextlabel("Revlabel","Change Gyro/Acc Direction"  ).setPosition(xServ-165,yServ+30).hide().moveTo("ServoSettings"); 
   TxtRevR   = controlP5.addTextlabel("Revtx","Change Dir in TX To Match"  ).setPosition(xServ-170,yServ+LabelPos).hide().moveTo("ServoSettings"); 

//****************** End of FlyingWng & TRI ********************
}

// ServoGraphics For Heli 120 && 90
if(multiType == HELI_120_CCPM || multiType == HELI_90_DEG ){
controlP5.getTab("ServoSettings").show();
 TxtMin    = controlP5.addTextlabel("Minlabel","MIN").setPosition(xServ+85,yServ+20) .hide().moveTo("ServoSettings");
 TxtMax    = controlP5.addTextlabel("Maxlabel","MAX").setPosition(xServ+130,yServ+20).hide().moveTo("ServoSettings");
 TxtMids   = controlP5.addTextlabel("Mlabel","Offset servos").setPosition(xServ+190,yServ+20).hide().moveTo("ServoSettings");
 TxtRates  = controlP5.addTextlabel("label","Not Used").setPosition(xServ+75,yServ+20).hide().moveTo("ServoSettings");
 TxtRev    = controlP5.addTextlabel("txtlblRev","Servos").setPosition(xServ+10,yServ+20).hide().moveTo("ServoSettings");
 
// CCPM settings
 // Mixer Boxes
 int XPos=xServ+40;
 int YPos=yServ-0; 
 Mbox  = controlP5.addCheckBox("Wbox").setColorForeground(color(120)).setColorBackground(color(140)).setColorActive(color(255)).setColorLabel(color(255)).setSize(20, 10)
       .setItemsPerRow(3).setSpacingColumn(15).setSpacingRow(10).setPosition(XPos-180  ,YPos+50)
       .addItem("mix3_4", 254) .addItem("mix3_2", 254).addItem("mix3_1", 254)
       .addItem("mix4_4", 254) .addItem("mix4_2", 254).addItem("mix4_1", 254)
       .addItem("mix6_4", 254) .addItem("mix6_2", 254).addItem("mix6_1", 254)
       .moveTo("ServoSettings").hideLabels().hide();
 TxtMIX[0]   = controlP5.addTextlabel("mix1","NICK") .setPosition(XPos-220,YPos+50).moveTo("ServoSettings");
 TxtMIX[1]   = controlP5.addTextlabel("mix2","LEFT") .setPosition(XPos-220,YPos+70).moveTo("ServoSettings");
 TxtMIX[2]   = controlP5.addTextlabel("mix3","RIGHT").setPosition(XPos-220,YPos+90).moveTo("ServoSettings"); 
 TxtMIX[3]   = controlP5.addTextlabel("mix4","COLL") .setPosition(XPos-195,YPos+30).moveTo("ServoSettings");
 TxtMIX[4]   = controlP5.addTextlabel("mix5","NICK") .setPosition(XPos-155,YPos+30).moveTo("ServoSettings");
 TxtMIX[5]   = controlP5.addTextlabel("mix6","ROLL") .setPosition(XPos-115,YPos+30).moveTo("ServoSettings");
 for (i=0;i<6;i++) { TxtMIX[i].hide();}

  //******************End of Heli********************
}
// Common Graphics for servos
  Step =0;
  for (i=0;i<8;i++) { // TODO Something
   BtServo[i] = controlP5.addButton("CHb"+i,1,xServ-30,yServ+40+20*i,60,12).setColorBackground(green_).setLabel("Servo "+i).hide().moveTo("ServoSettings");
    
    checkboxRev[i] =  controlP5.addCheckBox("cbR"+i).moveTo("ServoSettings");
    checkboxRev[i].setPosition(xServ+70,yServ+40+20*i)
    .setColorActive(color(255)).setColorBackground(color(120))
    .setItemHeight(10).setItemWidth(20).hide()
    .moveTo("ServoSettings")
    ;
   
    RateSlider[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("Rate"+i,0,xServ+70,yServ+40+i*20,100,14));
    RateSlider[i].setColorBackground(yellow_);RateSlider[i].setDirection(Controller.HORIZONTAL);
    RateSlider[i].setDecimalPrecision(0);RateSlider[i].setMultiplier(1);RateSlider[i].setMin(0).setMax(125).hide().moveTo("ServoSettings");
  }
  
  for (i=0;i<4;i++) BtAUX[i] = controlP5.addButton("Cau"+i,1,xServ-100,yServ+40+20*i,60,12).setColorBackground(red_).setLabel("  AUX "+(i+1)).moveTo("ServoSettings").hide();
  BtAUX[4] = controlP5.addButton("Cau4" ,1,xServ-100,yServ+120,60,12).setColorBackground(blue_).setLabel("Disable").moveTo("ServoSettings").hide();
  
 //************************ End of servoGrapics********************** 
}

void create_checkboxes(String[] names) {
  /* destroy old buttons */
  for (int i=0; i<CHECKBOXITEMS; i++) {
    buttonCheckbox[i].remove();
    checkbox[i].remove();
  }
  int i=0;
  /* create new list entries and buttons */
  checkbox = new CheckBox[names.length];
  buttonCheckbox = new Button[names.length];
  activation = new int[names.length];
  for (String name : names) {
  if ( name.equals("CAMTRIG")) camTrigger=true;
  if ( name.equals("CAMSTAB") || name.equals("CAMTRIG")) gimbal=true;
    buttonCheckbox[i] = controlP5.addButton("bcb"+i,1,xBox-30,yBox+20+13*i,68,12).setColorBackground(red_).setLabel(name);
    checkbox[i] =  controlP5.addCheckBox("cb"+i,xBox+40,yBox+20+13*i).setColorActive(color(255)).setColorBackground(color(120)).
    setItemsPerRow(12).setSpacingColumn(10).setLabel("");
    for (int j=1; j<=12; j++) checkbox[i].addItem(i + "_cb_" + j, j).hideLabels();
    i++;
  }
  CHECKBOXITEMS = names.length;
}

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
  // not compatible currently with reconnect feature
  // http://www.multiwii.com/forum/viewtopic.php?f=8&t=4077&p=41692#p41692
  /*
  if(shortName.startsWith("/dev/")) shortName = shortName.substring(5);  
  if(shortName.startsWith("tty.")) shortName = shortName.substring(4); // get rid of leading tty. part of device name
  if(shortName.length()>maxlen) shortName = shortName.substring(0,(maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  if(shortName.startsWith("cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
  */
}

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}

void setup() {
  // Trying to make both worlds happy..
  if( P3D == OPENGL ) pVersion = 2.0; else pVersion = 1.5;

  size(windowsX,windowsY,OPENGL);
  frameRate(20);

  font8 = createFont("Arial bold",8,false);
  font9 = createFont("Arial bold",9,false);
  font12 = createFont("Arial bold",12,false);
  font15 = createFont("Arial bold",15,false);

  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font12);
  addTabs();  

  g_graph  = new cGraph(xGraph+110,yGraph, 480, 200);
  
  // Baud list items
  baudListbox = controlP5.addListBox("baudList",5,95+tabHeight,110,240).moveTo("Config"); // make a listbox with available Baudrates
  baudListbox.captionLabel().set("BAUD_RATE");
  baudListbox.setColorBackground(red_);
  baudListbox.setBarHeight(17);
  
  baudListbox.addItem("9600"  ,9600); // addItem(name,value)
  baudListbox.addItem("14400" ,14400);
  baudListbox.addItem("19200" ,19200);
  baudListbox.addItem("28800" ,28800);
  baudListbox.addItem("38400" ,38400);
  baudListbox.addItem("57600" ,57600);
  baudListbox.addItem("115200",115200);
  
  // make a listbox and populate it with the available comm ports
  commListbox = controlP5.addListBox("portComList",5,105+tabHeight,110,120); 
  commListbox.captionLabel().set("PORT COM");
  commListbox.setColorBackground(red_);
  commListbox.setBarHeight(17);
  
  for( i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    //if( pn.startsWith("/dev/ttyUSB") ) {
      if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
      commListMax = i;
    //}
  }
  commListbox.addItem("Close Comm",++commListMax); // addItem(name,value)
  // text label for which comm port selected
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,65+tabHeight); // textlabel(name,text,x,y)
  // Information textlabels
  TxtInfo   = controlP5.addTextlabel("SInf","Remember To Save Changes to Eeprom!!") .setPosition(xServ-30, yServ+210).hide().moveTo("ServoSettings");
  TxtInfo1  = controlP5.addTextlabel("xInf","Grey Values Is Set As #define In Config.h!!") .setPosition(xServ+0, yServ+210).moveTo("Config");
  TxtInfo2  = controlP5.addTextlabel("gInf","Green Values Can Be Changed Press Write To Save!!") .setPosition(xServ+0, yServ+190).moveTo("Config");
  TxtInfo3  = controlP5.addTextlabel("CsInf","Remember To Activate CAMSTAB!!") .setPosition(xServ+10, yServ+20).hide().moveTo("ServoSettings");
  TxtInfo4  = controlP5.addTextlabel("CtInf","Remember To Activate CAMTRIG!!") .setPosition(xServ+10, yServ+20).hide().moveTo("ServoSettings");
  Links     = controlP5.addTextlabel("LinkInf","Some Useful Webpages!!") .setPosition(xServ+100, yServ+20).moveTo("Config");
  
 TxtInfoMotors1  = controlP5.addTextlabel("motInf1","This is a function for Balancing Propellors Dynamicly\n"+
"Select motor(s) and control throttle.").setPosition(xServ-200, yServ+10).moveTo("Motors");

  buttonSAVE   = controlP5.addButton("bSAVE",1,5,45+tabHeight,40,19).setLabel("SAVE").setColorBackground(red_);
  buttonIMPORT = controlP5.addButton("bIMPORT",1,50,45+tabHeight,40,19).setLabel("LOAD").setColorBackground(red_);
 
  btnQConnect = controlP5.addButton("bQCONN",1,xGraph+0,yGraph-105,100,19).setLabel("  ReConnect").setColorBackground(red_);
  buttonSTART  = controlP5.addButton("bSTART",1,xGraph+110,yGraph-25,45,19).setLabel("START").setColorBackground(red_);
  buttonSTOP   = controlP5.addButton("bSTOP",1,xGraph+160,yGraph-25,45,19).setLabel("STOP").setColorBackground(red_);

  buttonAcc   = controlP5.addButton("bACC",1,xButton,yButton,45,15).setColorBackground(red_).setLabel("ACC");
  buttonBaro  = controlP5.addButton("bBARO",1,xButton+50,yButton,45,15).setColorBackground(red_).setLabel("BARO");
  buttonMag   = controlP5.addButton("bMAG",1,xButton+100,yButton,45,15).setColorBackground(red_).setLabel("MAG");
  buttonGPS   = controlP5.addButton("bGPS",1,xButton,yButton+17,45,15).setColorBackground(red_).setLabel("GPS");
  buttonSonar = controlP5.addButton("bSonar",1,xButton+50,yButton+17,45,15).setColorBackground(red_).setLabel("SONAR");
  buttonOptic = controlP5.addButton("bOptic",1,xButton+100,yButton+17,45,15).setColorBackground(grey_).setLabel("OPTIC");

  color c,black;
  black = color(0,0,0);
  int xo = xGraph-7;
  int x = xGraph+40;
  int y1= yGraph+10;  //ACC
  int y2= yGraph+55;  //GYRO
  int y5= yGraph+100; //MAG
  int y3= yGraph+150; //ALT
  int y4= yGraph+165; //HEAD
  int y7= yGraph+185; //GPS
  int y6= yGraph+205; //DEBUG

  tACC_ROLL =       controlP5.addToggle("ACC_ROLL",true,x,y1+10,20,10).setColorActive(color(255, 0, 0)).setColorBackground(black).setLabel(""); 
  tACC_PITCH =      controlP5.addToggle("ACC_PITCH",true,x,y1+20,20,10).setColorActive(color(0, 255, 0)).setColorBackground(black).setLabel(""); 
  tACC_Z =          controlP5.addToggle("ACC_Z",true,x,y1+30,20,10).setColorActive(color(0, 0, 255)).setColorBackground(black).setLabel(""); 
  tGYRO_ROLL =      controlP5.addToggle("GYRO_ROLL",true,x,y2+10,20,10).setColorActive(color(200, 200, 0)).setColorBackground(black).setLabel(""); 
  tGYRO_PITCH =     controlP5.addToggle("GYRO_PITCH",true,x,y2+20,20,10).setColorActive(color(0, 255, 255)).setColorBackground(black).setLabel(""); 
  tGYRO_YAW =       controlP5.addToggle("GYRO_YAW",true,x,y2+30,20,10).setColorActive(color(255, 0, 255)).setColorBackground(black).setLabel(""); 
  tBARO   =         controlP5.addToggle("BARO",true,x,y3 ,20,10).setColorActive(color(125, 125, 125)).setColorBackground(black).setLabel(""); 
  tHEAD   =         controlP5.addToggle("HEAD",true,x,y4 ,20,10).setColorActive(color(225, 225, 125)).setColorBackground(black).setLabel(""); 
  tMAGX   =         controlP5.addToggle("MAGX",true,x,y5+10,20,10).setColorActive(color(50, 100, 150)).setColorBackground(black).setLabel(""); 
  tMAGY   =         controlP5.addToggle("MAGY",true,x,y5+20,20,10).setColorActive(color(100, 50, 150)).setColorBackground(black).setLabel(""); 
  tMAGZ   =         controlP5.addToggle("MAGZ",true,x,y5+30,20,10).setColorActive(color(150, 100, 50)).setColorBackground(black).setLabel(""); 
  tDEBUG1 =         controlP5.addToggle("DEBUG1",true,x+70,y6,20,10) .setColorActive(color(200, 50, 0)).setColorBackground(black).setLabel("").setValue(0);
  tDEBUG2 =         controlP5.addToggle("DEBUG2",true,x+190,y6,20,10).setColorActive(color(0, 200, 50)).setColorBackground(black).setLabel("").setValue(0);
  tDEBUG3 =         controlP5.addToggle("DEBUG3",true,x+310,y6,20,10).setColorActive(color(50, 0, 200)).setColorBackground(black).setLabel("").setValue(0);
  tDEBUG4 =         controlP5.addToggle("DEBUG4",true,x+430,y6,20,10).setColorActive(color(150, 100, 50)).setColorBackground(black).setLabel("").setValue(0);

  controlP5.addTextlabel( "alarmLabel", "Alarm:", xGraph -5, yGraph -32);

  controlP5.addTextlabel("acclabel","ACC",xo,y1 -4);
  controlP5.addTextlabel("accrolllabel","   ROLL",xo,y1+10).setFont(font9);
  controlP5.addTextlabel("accpitchlabel","   PITCH",xo,y1+20).setFont(font9);
  controlP5.addTextlabel("acczlabel","   Z",xo,y1+30).setFont(font9);
  controlP5.addTextlabel("gyrolabel","GYRO",xo,y2 -4);
  controlP5.addTextlabel("gyrorolllabel","   ROLL",xo,y2+10).setFont(font9);
  controlP5.addTextlabel("gyropitchlabel","   PITCH",xo,y2+20).setFont(font9);
  controlP5.addTextlabel("gyroyawlabel","   YAW",xo,y2+30).setFont(font9);
  controlP5.addTextlabel("maglabel","MAG",xo,y5 -4);
  controlP5.addTextlabel("magrolllabel","   ROLL",xo,y5+10).setFont(font9);
  controlP5.addTextlabel("magpitchlabel","   PITCH",xo,y5+20).setFont(font9);
  controlP5.addTextlabel("magyawlabel","   YAW",xo,y5+30).setFont(font9);
  controlP5.addTextlabel("altitudelabel","ALT",xo,y3 -4);
  controlP5.addTextlabel("headlabel","HEAD",xo,y4 -4);
  controlP5.addTextlabel("debug1","debug1",x+90,y6 -2).setFont(font9);
  controlP5.addTextlabel("debug2","debug2",x+210,y6 -2).setFont(font9);
  controlP5.addTextlabel("debug3","debug3",x+330,y6 -2).setFont(font9);
  controlP5.addTextlabel("debug4","debug4",x+450,y6 -2).setFont(font9);

  axSlider      =    controlP5.addSlider("axSlider",-1000,+1000,0,x+20,y1+10,50,10).setDecimalPrecision(0).setLabel("");
  aySlider      =    controlP5.addSlider("aySlider",-1000,+1000,0,x+20,y1+20,50,10).setDecimalPrecision(0).setLabel("");
  azSlider      =    controlP5.addSlider("azSlider",-1000,+1000,0,x+20,y1+30,50,10).setDecimalPrecision(0).setLabel("");
  gxSlider      =    controlP5.addSlider("gxSlider",-5000,+5000,0,x+20,y2+10,50,10).setDecimalPrecision(0).setLabel("");
  gySlider      =    controlP5.addSlider("gySlider",-5000,+5000,0,x+20,y2+20,50,10).setDecimalPrecision(0).setLabel("");
  gzSlider      =    controlP5.addSlider("gzSlider",-5000,+5000,0,x+20,y2+30,50,10).setDecimalPrecision(0).setLabel("");
  altSlider     =    controlP5.addSlider("altSlider",-30000,+30000,0,x+20,y3 ,50,10).setDecimalPrecision(2).setLabel("");
  headSlider    =    controlP5.addSlider("headSlider",-200,+200,0,x+20,y4  ,50,10).setDecimalPrecision(0).setLabel("");
  magxSlider    =    controlP5.addSlider("magxSlider",-5000,+5000,0,x+20,y5+10,50,10).setDecimalPrecision(0).setLabel("");
  magySlider    =    controlP5.addSlider("magySlider",-5000,+5000,0,x+20,y5+20,50,10).setDecimalPrecision(0).setLabel("");
  magzSlider    =    controlP5.addSlider("magzSlider",-5000,+5000,0,x+20,y5+30,50,10).setDecimalPrecision(0).setLabel("");
  debug1Slider  =    controlP5.addSlider("debug1Slider",-32768,+32767,0,x+130,y6,50,10).setDecimalPrecision(0).setLabel("");
  debug2Slider  =    controlP5.addSlider("debug2Slider",-32768,+32767,0,x+250,y6,50,10).setDecimalPrecision(0).setLabel("");
  debug3Slider  =    controlP5.addSlider("debug3Slider",-32768,+32767,0,x+370,y6,50,10).setDecimalPrecision(0).setLabel("");
  debug4Slider  =    controlP5.addSlider("debug4Slider",-32768,+32767,0,x+490,y6,50,10).setDecimalPrecision(0).setLabel("");

  for( i=0;i<PIDITEMS;i++) {
    confP[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP"+i,0,xParam+40,yParam+20+i*17,30,14));
    confP[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confI[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI"+i,0,xParam+75,yParam+20+i*17,40,14));
    confI[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(3).setMultiplier(0.001).setMax(0.250);
    confD[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD"+i,0,xParam+120,yParam+20+i*17,30,14));
    confD[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(0).setMultiplier(1).setMax(100);
  }
  
  for( i=0;i<7;i++) {
    confINF[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confINF"+i,0,xParam+10,yParam+10+i*35,60,14));
    confINF[i].setColorBackground(green_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(0).setMultiplier(5).setMax(100).moveTo("Config").hide();
  }
  confINF[0].setLabel("Minthrottle").setMin(1000).setMax(1500).moveTo("Config");
  confINF[1].setLabel("MAXhrottle").setMin(1500).setMax(2000).moveTo("Config");
  confINF[2].setLabel("Min command").setMin(900).setMax(1200).moveTo("Config");
  confINF[3].setLabel("FS Thr").setMin(1200).setMax(2000).moveTo("Config");
  confINF[4].setLabel("Armed count").setMin(0).setMax(10000).setColorBackground(red_).moveTo("Config");
  confINF[5].setLabel("Acc ArmedTime").setMin(0).setMax(360000).setColorBackground(red_).moveTo("Config");
  confINF[6].setLabel("MAG Decl").setMin(-30).setMax(30).setDecimalPrecision(1).setMultiplier(.1).moveTo("Config");
 
  confI[8].hide();confD[8].hide();confD[4].hide();
  confP[9].hide();confI[9].hide();confD[9].hide();
  //change bounds for POS-4 POSR-5 and NAV-6
  confP[4].setDecimalPrecision(2).setMultiplier(0.01).setMax(5);
  confI[4].setDecimalPrecision(1).setMultiplier(0.1) .setMax(2.5);
  
  confP[5].setDecimalPrecision(1).setMultiplier(0.1) .setMax(25);
  confI[5].setDecimalPrecision(2).setMultiplier(0.01).setMax(2.5);
  confD[5].setDecimalPrecision(3).setMultiplier(.001).setMax(.250);
  
  confP[6].setDecimalPrecision(1).setMultiplier(0.1) .setMax(25);
  confI[6].setDecimalPrecision(2).setMultiplier(0.01).setMax(2.5);
  confD[6].setDecimalPrecision(3).setMultiplier(.001).setMax(.250);

  rollPitchRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("rollPitchRate",0,xParam+160,yParam+30,30,14)).setDecimalPrecision(2);
  rollPitchRate.setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_).setMultiplier(0.01);
  
  yawRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("yawRate",0,xParam+160,yParam+54,30,14)).setDecimalPrecision(2);
  yawRate.setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_).setMultiplier(0.01); 
  
  dynamic_THR_PID = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("dynamic_THR_PID",0,xParam+215,yParam+22,30,14)).setDecimalPrecision(2);
  dynamic_THR_PID.setMultiplier(0.01).setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_);

  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+40,yParam+220,30,14).setDecimalPrecision(2).setMultiplier(0.01).setLabel("");
  confRC_RATE.setDirection(Controller.HORIZONTAL).setMin(0).setMax(2.5).setColorBackground(red_);
  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+40,yParam+237,30,14).setDecimalPrecision(2).setMultiplier(0.01).setLabel("");
  confRC_EXPO.setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_);

  confSetting = controlP5.addNumberbox("_SETTING",0,xParam+2,yParam+2,30,14).setDecimalPrecision(0).setMultiplier(1).setLabel("")
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(2).setColorBackground(red_);//.hide();

  confSelectSetting = controlP5.addNumberbox("S_SETTING",0,xParam+520,yParam+260,30,14).setDecimalPrecision(0).setMultiplier(1).setLabel("")
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(2).setColorBackground(red_);//.hide();

  throttle_MID = controlP5.addNumberbox("T MID",0.5,xParam+40,yParam+180,30,14).setDecimalPrecision(2).setMultiplier(0.01).setLabel("")
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_);
  
  throttle_EXPO = controlP5.addNumberbox("T EXPO",0,xParam+40,yParam+197,30,14).setDecimalPrecision(2).setMultiplier(0.01).setLabel("")
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(1).setColorBackground(red_);
  
  buttonREAD =          controlP5.addButton("READ" , 1, xParam+5,   yParam+260, 50, 16)  .setColorBackground(red_);
  buttonRESET =         controlP5.addButton("RESET", 1, xParam+60,  yParam+260, 60, 16)  .setColorBackground(red_);
  buttonWRITE =         controlP5.addButton("WRITE", 1, xParam+290, yParam+260, 50, 16)  .setColorBackground(red_);
  buttonSERVO =         controlP5.addButton("SERVO", 1, xParam+5,   yParam+220, 55, 16)  .setColorBackground(red_).hide().moveTo("ServoSettings");
  buttonWing =          controlP5.addButton("WING" , 1, xParam+5,   yParam+220, 55, 16)  .setColorBackground(red_).hide().moveTo("ServoSettings");
  buttonExport=         controlP5.addButton("Eport_Servo",1,  xParam+5,  yParam+10, 100,19)   .setLabel("Export to file").setColorBackground(green_).moveTo("ServoSettings").hide();
 
  SaveSERVO =           controlP5.addButton("SAVE_Servo", 1 , xParam+290, yParam+260, 55, 16).setColorBackground(green_).hide().setLabel("  Save").moveTo("ServoSettings");
  SaveWing  =           controlP5.addButton("SAVE_WING",  1 , xParam+290, yParam+260, 55, 16).setColorBackground(green_).hide().setLabel("  Save").moveTo("ServoSettings");
  buttonLIVE=           controlP5.addButton("LIVE_SERVO", 1 , xParam+65, yParam+220, 75, 16).setColorBackground(red_).setLabel("Go Live").hide().moveTo("ServoSettings");
  buttonCALIBRATE_ACC = controlP5.addButton("CALIB_ACC",  1,  xParam+210, yParam+260, 70, 16).setColorBackground(red_);
  buttonCALIBRATE_MAG = controlP5.addButton("CALIB_MAG",  1,  xParam+130, yParam+260, 73, 16).setColorBackground(red_);
  buttonSETTING =       controlP5.addButton("SETTING"  ,  1,  xParam+410, yParam+260, 105, 16) .setColorBackground(red_).setLabel("SELECT SETTING");//.hide();
  buttonGimbal =        controlP5.addButton("GIMBAL"   ,  1,  xParam+5,   yParam+200,55,16).setColorBackground(green_).hide().moveTo("ServoSettings");
  btnTrigger   =        controlP5.addButton("TRIGGER"  , 1,   xParam+65,   yParam+200,75,16).setColorBackground(green_).hide().moveTo("ServoSettings");
  btMagDecl    =        controlP5.addButton("MagDecl"  ,  1,  xParam+350, yParam+100, 180, 16).setColorBackground(grey_).moveTo("Config").setLabel("magnetic-declination.com");
  btMWiiHome   =        controlP5.addButton("MWiiHome" ,  1,  xParam+350, yParam+60, 180, 16).setColorBackground(grey_).moveTo("Config").setLabel("MUltiWii Forum");
  btDownloads  =        controlP5.addButton("MWiiGet"  ,  1,  xParam+350, yParam+80, 180, 16).setColorBackground(grey_).moveTo("Config").setLabel("MUltiWii Downloads");
  

// Sliders for Transmitter
  TX_StickSlider[RCRoll ] =  controlP5.addSlider("Roll", 900,2100,1500,xRC,yRC+15,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCPitch] =  controlP5.addSlider("Pitch",900,2100,1500,xRC,yRC+30,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCThro ] =  controlP5.addSlider("Throt",900,2100,1500,xRC,yRC,100,10)     .setDecimalPrecision(0);
  TX_StickSlider[RCYaw ]  =  controlP5.addSlider("Yaw",  900,2100,1500,xRC,yRC+45,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCAUX1]  =  controlP5.addSlider("AUX1", 1000,2000,1500,xRC,yRC+60,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX2]  =  controlP5.addSlider("AUX2", 1000,2000,1500,xRC,yRC+75,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX3]  =  controlP5.addSlider("AUX3", 1000,2000,1500,xRC,yRC+90,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX4]  =  controlP5.addSlider("AUX4", 1000,2000,1500,xRC,yRC+105,100,10).setDecimalPrecision(0);

  motorControlSlider      = controlP5.addSlider("Motors",1000,2000,1500,xParam+410, yParam,90,30)   .setDecimalPrecision(0).hide();

  for( i=0;i<8;i++) {
    motToggle[i]     = controlP5.addToggle("M"+i,false,xMot-100,yMot+15,20,15).moveTo("Motors").hide();
    motSlider[i]     = controlP5.addSlider("motSlider"+i,900,2000,1500,0,0,10,100).setDecimalPrecision(0).hide();
    servoSliderH[i]  = controlP5.addSlider("ServoH"+i,1000,2000,1500,0,0,100,10).setDecimalPrecision(0).hide();
    servoSliderV[i]  = controlP5.addSlider("ServoV"+i,1000,2000,1500,0,0,10,100).setDecimalPrecision(0).hide();
  }
  for( i=0;i<6;i++) {
    VBat[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("VBat"+i,0,xParam+120,yParam+20+i*35,60,14));
    VBat[i].setColorBackground(red_).setMin(0).setMax(20).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(1).moveTo("Config").hide();
  }
  VBat[0].setCaptionLabel("VBatscale").setDecimalPrecision(0).setMin(0).setMax(200);
  VBat[1].setCaptionLabel("Warning 1").setMultiplier(0.1);
  VBat[2].setCaptionLabel("Warning 2").setMultiplier(0.1);
  VBat[3].setCaptionLabel("Critical").setMultiplier(0.1);
  VBat[4].setCaptionLabel("Volt").setColorBackground(red_).setDecimalPrecision(2);
  VBat[5].setCaptionLabel("RSSI").setColorBackground(red_).setDecimalPrecision(0).setMin(0).setMax(1023);
   
  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+515,yGraph,75,20).setLabel("");// GraphScaler
 
  confPowerTrigger = controlP5.addNumberbox("",0,xGraph+50,yGraph-29,40,14).setDecimalPrecision(0).setMultiplier(10)
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(65535).setColorBackground(red_);
  
  Tooltips();
}     /************* End of setup() *************/



/******************************* Multiwii Serial Protocol **********************/
private static final String MSP_HEADER = "$M<";

private static final int
  MSP_IDENT                =100,
  MSP_STATUS               =101,
  MSP_RAW_IMU              =102,
  MSP_SERVO                =103,
  MSP_MOTOR                =104,
  MSP_RC                   =105,
  MSP_RAW_GPS              =106,
  MSP_COMP_GPS             =107,
  MSP_ATTITUDE             =108,
  MSP_ALTITUDE             =109,
  MSP_ANALOG               =110,
  MSP_RC_TUNING            =111,
  MSP_PID                  =112,
  MSP_BOX                  =113,
  MSP_MISC                 =114,
  MSP_MOTOR_PINS           =115,
  MSP_BOXNAMES             =116,
  MSP_PIDNAMES             =117,
  MSP_SERVO_CONF           =120,
    
  
  MSP_SET_RAW_RC           =200,
  MSP_SET_RAW_GPS          =201,
  MSP_SET_PID              =202,
  MSP_SET_BOX              =203,
  MSP_SET_RC_TUNING        =204,
  MSP_ACC_CALIBRATION      =205,
  MSP_MAG_CALIBRATION      =206,
  MSP_SET_MISC             =207,
  MSP_RESET_CONF           =208,
  MSP_SELECT_SETTING       =210,
  MSP_SET_HEAD             =211, // Not used
  MSP_SET_SERVO_CONF       =212,
  MSP_SET_MOTOR            =214,
  
  
  MSP_BIND                 =241,

  MSP_EEPROM_WRITE         =250,
  
  MSP_DEBUGMSG             =253,
  MSP_DEBUG                =254
;

public static final int
  IDLE = 0,
  HEADER_START = 1,
  HEADER_M = 2,
  HEADER_ARROW = 3,
  HEADER_SIZE = 4,
  HEADER_CMD = 5,
  HEADER_ERR = 6
;

int c_state = IDLE;
boolean err_rcvd = false;

byte checksum=0;
byte cmd;
int offset=0, dataSize=0;
byte[] inBuf = new byte[256];


int p;
int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
int read8()  {return  inBuf[p++]&0xff;}

int mode;
boolean toggleRead = false,toggleReset = false,toggleCalibAcc = false,toggleCalibMag = false,toggleWrite = false,
        toggleRXbind = false,toggleSetSetting = false,toggleVbat=true,toggleMotor=false,motorcheck=true;

//send msp without payload
private List<Byte> requestMSP(int msp) {
  return  requestMSP( msp, null);
}

//send multiple msp without payload
private List<Byte> requestMSP (int[] msps) {
  List<Byte> s = new LinkedList<Byte>();
  for (int m : msps) {
    s.addAll(requestMSP(m, null));
  }
  return s;
}

//send msp with payload
private List<Byte> requestMSP (int msp, Character[] payload) {
  if(msp < 0) {
   return null;
  }
  List<Byte> bf = new LinkedList<Byte>();
  for (byte c : MSP_HEADER.getBytes()) {
    bf.add( c );
  }
  
  byte checksum=0;
  byte pl_size = (byte)((payload != null ? int(payload.length) : 0)&0xFF);
  bf.add(pl_size);
  checksum ^= (pl_size&0xFF);
  
  bf.add((byte)(msp & 0xFF));
  checksum ^= (msp&0xFF);
  
  if (payload != null) {
    for (char c :payload){
      bf.add((byte)(c&0xFF));
      checksum ^= (c&0xFF);
    }
  }
  bf.add(checksum);
  return (bf);
}

void sendRequestMSP(List<Byte> msp) {
  byte[] arr = new byte[msp.size()];
  int i = 0;
  for (byte b: msp) {
    arr[i++] = b;
  }
  g_serial.write(arr); // send the complete byte sequence in one go
}

public void evaluateCommand(byte cmd, int dataSize) {
  int i;
  int icmd = (int)(cmd&0xFF);
  switch(icmd) {
    case MSP_IDENT:
        version = read8();
        multiType = read8();
        read8(); // MSP version
        multiCapability = read32();// capability
        if ((multiCapability&1)>0) {buttonRXbind = controlP5.addButton("bRXbind",1,10,yGraph+205-10,55,10); buttonRXbind.setColorBackground(blue_);buttonRXbind.setLabel("RX Bind");}
        if ((multiCapability&4)>0) controlP5.addTab("Motors").show();
        if ((multiCapability&8)>0) flaps=true;
        if (!GraphicsInited)  create_ServoGraphics();
       break;

    case MSP_STATUS:
        cycleTime = read16();
        i2cError = read16();
        present = read16();
        mode = read32();
        if ((present&1) >0) {buttonAcc.setColorBackground(green_);} else {buttonAcc.setColorBackground(red_);tACC_ROLL.setState(false); tACC_PITCH.setState(false); tACC_Z.setState(false);}
        if ((present&2) >0) {buttonBaro.setColorBackground(green_);} else {buttonBaro.setColorBackground(red_); tBARO.setState(false); }
        if ((present&4) >0) {buttonMag.setColorBackground(green_); Mag_=true;} else {buttonMag.setColorBackground(red_); tMAGX.setState(false); tMAGY.setState(false); tMAGZ.setState(false);}
        if ((present&8) >0) {buttonGPS.setColorBackground(green_);} else {buttonGPS.setColorBackground(red_); tHEAD.setState(false);}
        if ((present&16)>0) {buttonSonar.setColorBackground(green_);} else {buttonSonar.setColorBackground(red_);}

        for(i=0;i<CHECKBOXITEMS;i++) {if ((mode&(1<<i))>0) buttonCheckbox[i].setColorBackground(green_); else buttonCheckbox[i].setColorBackground(red_);}
        confSetting.setValue(read8());
        confSetting.setColorBackground(green_);
        break;
    case MSP_RAW_IMU:
        ax = read16();ay = read16();az = read16();
        if (ActiveTab=="Motors"){ // Show unfilterd values in graph.
          gx = read16();gy = read16();gz = read16();
         magx = read16();magy = read16();magz = read16(); 
         }else{
         gx = read16()/8;gy = read16()/8;gz = read16()/8;
         magx = read16()/3;magy = read16()/3;magz = read16()/3; 
       }break;
    case MSP_SERVO:for(i=0;i<8;i++) servo[i] = read16(); break;
    case MSP_MOTOR: for(i=0;i<8;i++){ mot[i] = read16();} 
        if (multiType == SINGLECOPTER)servo[7]=mot[0];
        if (multiType == DUALCOPTER){servo[7]=mot[0];servo[6]=mot[1];}
        break; 
    case MSP_RC:
        for(i=0;i<8;i++) {
          RCChan[i]=read16();
          TX_StickSlider[i].setValue(RCChan[i]);
        }
        break;
    case MSP_RAW_GPS:
        GPS_fix = read8();
        GPS_numSat = read8();
        GPS_latitude = read32();
        GPS_longitude = read32();
        GPS_altitude = read16();
        GPS_speed = read16(); break;
    case MSP_COMP_GPS:
        GPS_distanceToHome = read16();
        GPS_directionToHome = read16();
        GPS_update = read8(); break;
    case MSP_ATTITUDE:
        angx = read16()/10;angy = read16()/10;
        head = read16(); break;
    case MSP_ALTITUDE: alt = read32(); break;
    case MSP_ANALOG:
        bytevbat = read8();
        pMeterSum = read16();
        rssi = read16(); if(rssi!=0)VBat[5].setValue(rssi).show();  // rssi
        amperage = read16(); // amperage
        VBat[4].setValue(bytevbat/10.0);    // Volt
        break;
    case MSP_RC_TUNING:
        byteRC_RATE = read8();byteRC_EXPO = read8();byteRollPitchRate = read8();
        byteYawRate = read8();byteDynThrPID = read8();
        byteThrottle_MID = read8();byteThrottle_EXPO = read8();
        confRC_RATE.setValue(byteRC_RATE/100.0);
        confRC_EXPO.setValue(byteRC_EXPO/100.0);
        rollPitchRate.setValue(byteRollPitchRate/100.0);
        yawRate.setValue(byteYawRate/100.0);
        dynamic_THR_PID.setValue(byteDynThrPID/100.0);
        throttle_MID.setValue(byteThrottle_MID/100.0);
        throttle_EXPO.setValue(byteThrottle_EXPO/100.0);
        confRC_RATE.setColorBackground(green_);confRC_EXPO.setColorBackground(green_);rollPitchRate.setColorBackground(green_);
        yawRate.setColorBackground(green_);dynamic_THR_PID.setColorBackground(green_);
        throttle_MID.setColorBackground(green_);throttle_EXPO.setColorBackground(green_);
        updateModelMSP_SET_RC_TUNING();
        break;
    case MSP_ACC_CALIBRATION:break;
    case MSP_MAG_CALIBRATION:break;
    case MSP_PID:
        for(i=0;i<PIDITEMS;i++) {
          byteP[i] = read8();byteI[i] = read8();byteD[i] = read8();
          switch (i) {
           case 0:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 1:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 2:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 3:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 7:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 8:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           case 9:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);break;
           //Different rates fot POS-4 POSR-5 NAVR-6
           case 4:confP[i].setValue(byteP[i]/100.0);confI[i].setValue(byteI[i]/100.0);confD[i].setValue(byteD[i]/1000.0);break;
           case 5:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/100.0);confD[i].setValue(byteD[i]/1000.0);break;
           case 6:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/100.0);confD[i].setValue(byteD[i]/1000.0);break;
          }
          confP[i].setColorBackground(green_);confI[i].setColorBackground(green_);confD[i].setColorBackground(green_);
        }
        updateModelMSP_SET_PID();
        break;
    case MSP_BOX:
        for( i=0;i<CHECKBOXITEMS;i++) {
          activation[i] = read16();
          for(int aa=0;aa<12;aa++) {
            if ((activation[i]&(1<<aa))>0) {checkbox[i].activate(aa);}else {checkbox[i].deactivate(aa);}}} break;
    case MSP_BOXNAMES:
        create_checkboxes(new String(inBuf, 0, dataSize).split(";"));break;
    case MSP_PIDNAMES:
        /* TODO create GUI elements from this message */
        //System.out.println("Got PIDNAMES: "+new String(inBuf, 0, dataSize));
        break;
    case MSP_SERVO_CONF:
        Bbox.deactivateAll();
        // min:2 / max:2 / middle:2 / rate:1 
         for( i=0;i<8;i++){
           ServoMIN[i]   = read16(); 
           ServoMAX[i]   = read16(); 
           ServoMID[i]   = read16(); 
           servoRATE[i]  = read8() ;
         }
       if (multiType == AIRPLANE ) { // Airplane OK
         if(flaps) {
             //ServoSliderC[2].setMin(4).setMax(10);
             if(ServoMID[2]==4) {Cau0();}else if(ServoMID[2]==5) {Cau1();}else if(ServoMID[2]==6) {Cau2();}else if(ServoMID[2]==7){Cau3();}else{CauClear();}
         }
	 for( i=0;i<8;i++){
	   ServoSliderMIN[i].setValue(ServoMIN[i]); //Update sliders
	   ServoSliderMAX[i].setValue(ServoMAX[i]);
	   ServoSliderC[i].setValue(ServoMID[i]);
           if (servoRATE[i]>127){ // Reverse/Rate servos
             Bbox.deactivate(i); RateSlider[i].setValue(abs(servoRATE[i]-256));
           }else{
             Bbox.activate(i); RateSlider[i].setValue(abs(servoRATE[i]));
           }
         }

       } else if (multiType == FLYING_WING || multiType == TRI || multiType == BI  || multiType == DUALCOPTER  || multiType == SINGLECOPTER) { // FlyingWing & TRI & BI
         int nBoxes;
         for( i=0;i<8;i++){ //Update sliders
           ServoSliderMIN[i].setValue(ServoMIN[i]);
           ServoSliderMAX[i].setValue(ServoMAX[i]);
           ServoSliderC[i].setValue(ServoMID[i]);
           if (servoRATE[i]>127){ // Reverse/Rate servos
             wingDir[i]=-1; RateSlider[i].setValue((servoRATE[i]-256));
           }else{ wingDir[i]=1; RateSlider[i].setValue(abs(servoRATE[i])); } // Servo Direction
         }
         
         if(multiType == FLYING_WING) { //OK
           if ((servoRATE[3]&1)<1) {Wbox.deactivate(2);}else{Wbox.activate(2);} //
           if ((servoRATE[3]&2)<1) {Wbox.deactivate(0);}else{Wbox.activate(0);} //
           if ((servoRATE[4]&1)<1) {Wbox.deactivate(3);}else{Wbox.activate(3);} //
           if ((servoRATE[4]&2)<1) {Wbox.deactivate(1);}else{Wbox.activate(1);} //
          
           
         } else if(multiType == SINGLECOPTER) { //
           if ((servoRATE[3]&1)<1) {Wbox.deactivate(0);}else{Wbox.activate(0);} //
           if ((servoRATE[3]&2)<1) {Wbox.deactivate(1);}else{Wbox.activate(1);} //
           if ((servoRATE[4]&1)<1) {Wbox.deactivate(2);}else{Wbox.activate(2);} //
           if ((servoRATE[4]&2)<1) {Wbox.deactivate(3);}else{Wbox.activate(3);} //
           if ((servoRATE[5]&1)<1) {Wbox.deactivate(4);}else{Wbox.activate(4);} //
           if ((servoRATE[5]&2)<1) {Wbox.deactivate(5);}else{Wbox.activate(5);} //
           if ((servoRATE[6]&1)<1) {Wbox.deactivate(6);}else{Wbox.activate(6);} //
           if ((servoRATE[6]&2)<1) {Wbox.deactivate(7);}else{Wbox.activate(7);} //
          
           
         } else if(multiType == DUALCOPTER) { // OK
           if ((servoRATE[4]&1)<1) {Wbox.deactivate(0);}else{Wbox.activate(0);}
           if ((servoRATE[5]&1)<1) {Wbox.deactivate(1);}else{Wbox.activate(1);}
          
           
         }  else if (multiType == TRI) {// OK
             if ((servoRATE[5]&1)<1) {Wbox.deactivate(0);}else{Wbox.activate(0);}
           
         } else if( multiType == BI) {// OK
           if ((servoRATE[4]&2)<1) {Wbox.deactivate(0);}else{Wbox.activate(0);} // L
           if ((servoRATE[5]&2)<1) {Wbox.deactivate(1);}else{Wbox.activate(1);}
           if ((servoRATE[4]&1)<1) {Wbox.deactivate(2);}else{Wbox.activate(2);} // R
           if ((servoRATE[5]&1)<1) {Wbox.deactivate(3);}else{Wbox.activate(3);}
        }
        
       }else if (multiType == HELI_120_CCPM ||  multiType == HELI_90_DEG) {
         for( i=0;i<8;i++) { //Update sliders
           ServoSliderMIN[i].setValue(ServoMIN[i]);
           ServoSliderMAX[i].setValue(ServoMAX[i]);
           ServoSliderC[i].setValue(ServoMID[i]);
           
           if (servoRATE[i]>127){ // Reverse/Rate servos
             Bbox.deactivate(i); RateSlider[i].setValue((servoRATE[i]-256));
           }else{ Bbox.activate(i); RateSlider[i].setValue(abs(servoRATE[i]));}
         }
           if ((servoRATE[5]&1)<1) {Bbox.deactivate(5);}else{Bbox.activate(5);} // YawReverse
           if(multiType == HELI_120_CCPM) { //bbb
             if ((servoRATE[3]&1)<1) {Mbox.deactivate(2);}else{Mbox.activate(2);} // roll
             if ((servoRATE[3]&2)<1) {Mbox.deactivate(1);}else{Mbox.activate(1);} // nick
             if ((servoRATE[3]&4)<1) {Mbox.deactivate(0);}else{Mbox.activate(0);} // coll
             if ((servoRATE[4]&1)<1) {Mbox.deactivate(5);}else{Mbox.activate(5);} //
             if ((servoRATE[4]&2)<1) {Mbox.deactivate(4);}else{Mbox.activate(4);} //
             if ((servoRATE[4]&4)<1) {Mbox.deactivate(3);}else{Mbox.activate(3);} //
             if ((servoRATE[6]&1)<1) {Mbox.deactivate(8);}else{Mbox.activate(8);} //
             if ((servoRATE[6]&2)<1) {Mbox.deactivate(7);}else{Mbox.activate(7);} //
             if ((servoRATE[6]&4)<1) {Mbox.deactivate(6);}else{Mbox.activate(6);} //
           }
         
       }else if (multiType == PPM_TO_SERVO ) { // PPM_TO_SERVO
	 for( i=0;i<8;i++){
	   ServoSliderMIN[i].setValue(ServoMIN[i]); //Update sliders
	   ServoSliderMAX[i].setValue(ServoMAX[i]);
	   ServoSliderC[i]  .setValue(ServoMID[i]);
	 // Reverse/Rate servos
           if (servoRATE[i]>127){
             Bbox.deactivate(i); RateSlider[i].setValue(abs(servoRATE[i]-256));
           }else{Bbox.activate(i); RateSlider[i].setValue(abs(servoRATE[i]));}
         }
       }
       
       if (gimbal){
         if(!gimbalConfig)create_GimbalGraphics(); 
         // Switch beween Channels or Centerpos.
         if(ServoMID[0]>1200) {GimbalSlider[2] .setMin(1200).setMax(1700); }else{GimbalSlider[2] .setMin(0).setMax(12);}
         if(ServoMID[1]>1200) {GimbalSlider[6] .setMin(1200).setMax(1700); }else{GimbalSlider[6] .setMin(0).setMax(12);}
         if(ServoMID[2]>1000) {GimbalSlider[10].setMin(1000).setMax(30000);}else{GimbalSlider[10].setMin(0).setMax(12);}

         
         i=0;
         GimbalSlider[0] .setValue((int)ServoMIN[i]);
         GimbalSlider[1] .setValue((int)ServoMAX[i]);
         GimbalSlider[2] .setValue((int)ServoMID[i]);
         if (servoRATE[i]>127){ GimbalSlider[3].setValue((servoRATE[i]-256));
         }else{ GimbalSlider[3].setValue(abs(servoRATE[i]));}
         
         i=1;
         GimbalSlider[4] .setValue((int)ServoMIN[i]);
         GimbalSlider[5] .setValue((int)ServoMAX[i]);
         GimbalSlider[6] .setValue((int)ServoMID[i]);         
         if (servoRATE[i]>127){ GimbalSlider[7].setValue((servoRATE[i]-256));
         }else{GimbalSlider[7].setValue(abs(servoRATE[i]));}
         
         i=2;
         GimbalSlider[8] .setValue((int)ServoMIN[i]);
         GimbalSlider[9] .setValue((int)ServoMAX[i]);
         GimbalSlider[10].setValue((int)ServoMID[i]);
         GimbalSlider[11].setValue((int)servoRATE[i]);
       }
       if (camTrigger){
       if(ServoMID[2]>1200) {ServoSliderC[2].setMin(Centerlimits[0]).setMax(Centerlimits[1]);}else{ServoSliderC[2].setMin(0).setMax(12);}       
       }
       
       if(ExportServo) SAVE_SERVO_CONFIG(); // ServoConfig to file
   //******************************************************************************************
        break;
  
        
    case MSP_MISC:
        intPowerTrigger = read16(); // a
        
        //int minthrottle,maxthrottle,mincommand,FSthrottle,armedNum,lifetime,mag_decliniation ;
        for (i=0;i<4;i++) { MConf[i]= read16(); 
          confINF[i].setValue((int)MConf[i]).show(); 
        }
          if(MConf[3]<1000)confINF[3].hide();          
        
        // LOG_PERMANENT        
        MConf[4]= read16(); confINF[4].setValue((int)MConf[4]);//f
        MConf[5]= read32(); confINF[5].setValue((int)MConf[5]);//g        
        for (i=1;i<3;i++){confINF[i].setColorBackground(grey_).setMin((int)MConf[i]).setMax((int)MConf[i]);} //?
        
        // hide LOG_PERMANENT
        if(MConf[4]<1){confINF[5].hide();confINF[4].hide();}else{confINF[5].show();confINF[4].show();}
        
        //mag_decliniation
        MConf[6]= read16(); confINF[6].setValue((float)MConf[6]/10).show(); //h
        if(!Mag_)confINF[6].hide();        
 
        // VBAT
        int q = read8();if(toggleVbat){VBat[0].setValue(q).setColorBackground(green_);toggleVbat=false; // i
        for( i=1;i<4;i++) VBat[i].setValue(read8()/10.0).setColorBackground(green_);}  // j,k,l
        if(q > 1) for( i=0;i<5;i++) VBat[i].show();
        
        controlP5.addTab("Config").show();
        
        confPowerTrigger.setValue(intPowerTrigger);
        updateModelMSP_SET_MISC();
        break;
    case MSP_MOTOR_PINS:
        for( i=0;i<8;i++) {byteMP[i] = read8();}break;
    case MSP_DEBUGMSG:
        while(dataSize-- > 0) {
          char c = (char)read8();
          if (c != 0) {System.out.print( c );}
        }break;
    case MSP_DEBUG:
        debug1 = read16();debug2 = read16();debug3 = read16();debug4 = read16(); break;
    default:
        //println("Don't know how to handle reply "+icmd);
  }
}

private int present = 0;
int time,time2,time3,time4,time5,time6;

void draw() {
  List<Character> payload;
  int i,aa;
  float val,inter,a,b,h;
  int c;
  if (init_com==1 && graph_on==1) {
    time=millis();

    if ((time-time4)>40) {
      time4=time;
      accROLL.addVal(ax);accPITCH.addVal(ay);accYAW.addVal(az);gyroROLL.addVal(gx);gyroPITCH.addVal(gy);gyroYAW.addVal(gz);
      magxData.addVal(magx);magyData.addVal(magy);magzData.addVal(magz);
      altData.addVal(alt);headData.addVal(head);
      debug1Data.addVal(debug1);debug2Data.addVal(debug2);debug3Data.addVal(debug3);debug4Data.addVal(debug4);
    }
    
    if (!toggleRead && !toggleWrite && !toggleSetSetting ) {
      if ((time-time2)>40  ){
        time2=time;
        int[] requests = {MSP_STATUS, MSP_RAW_IMU, MSP_SERVO, MSP_MOTOR, MSP_RC,MSP_DEBUG};
        sendRequestMSP(requestMSP(requests));   
      }
      if ((time-time3)>25 ) {
        time3=time;
        sendRequestMSP(requestMSP(MSP_ATTITUDE));
      }
      if ((time-time5)>100 ) {
        time5=time;
        int[] requests = { MSP_ALTITUDE};
        sendRequestMSP(requestMSP(requests));
      }
      if ((time-time6)>200 ) {
        time6=time;
        int[] requests = { MSP_RAW_GPS, MSP_COMP_GPS, MSP_ANALOG};
        sendRequestMSP(requestMSP(requests));
        
        if (toggleMotor){
          payload = new ArrayList<Character>();
          for( i=0;i<8;i++){
            if (motToggle[i].getState()) {
              payload.add(char (int(motorControlSlider.getValue()) % 256) ); payload.add(char (int(motorControlSlider.getValue()) / 256)  );
            } else {
              payload.add(char (1000 % 256) ); payload.add(char (1000 / 256)  );
            }
          }
          sendRequestMSP(requestMSP(MSP_SET_MOTOR,payload.toArray( new Character[payload.size()]) ));
        }
      }
      if (!toggleLive) {
	buttonLIVE.setColorBackground(red_).setLabel("   Go Live");
        buttonRESET.show();
        controlP5.getTooltip().register("LIVE_SERVO","Enable Live changes to the Servos.") ;
      }else{
        buttonLIVE.setColorBackground(green_).setLabel("      Live");
        buttonRESET.hide();
        buttonExport.show();
        controlP5.getTooltip().register("LIVE_SERVO","Disable Live changes to the Servos.") ;
        if (multiType == FLYING_WING ||multiType == TRI  ||multiType == BI || toggleGimbal || toggleTrigger  || multiType == DUALCOPTER  || multiType == SINGLECOPTER)toggleWriteWingLive=true;
        if (multiType == HELI_120_CCPM || multiType == HELI_90_DEG ) toggleWriteServoLive=true;
        if (multiType == AIRPLANE || multiType == PPM_TO_SERVO ) toggleWriteServoLive=true;
      }
    }
    if (toggleReset) {
      toggleReset=false;
      toggleRead=true;
      sendRequestMSP(requestMSP(MSP_RESET_CONF));
    }
/*****************************************************************/
    

    if (toggleRead) {
      if (!toggleLive &&( ActiveTab=="default" ||  ActiveTab =="Config")){// && ActiveTab=="default".. Because of checksum error on MSP_PID with servosTab
        int[] requests = {MSP_IDENT ,MSP_BOXNAMES, MSP_RC_TUNING, MSP_PID, MSP_MOTOR_PINS,MSP_BOX,MSP_MISC};  // MSP_PIDNAMES, MSP_SERVO_CONF
        sendRequestMSP(requestMSP(requests));
      }
      if(GraphicsInited  ){ // Don't call for servo conf before Graphics is created
        int[] reques = { MSP_SERVO_CONF};
        sendRequestMSP(requestMSP(reques));
      }
      buttonWRITE.setColorBackground(green_);
      buttonSERVO.setColorBackground(green_);
      buttonSETTING.setColorBackground(green_);
      confSelectSetting.setColorBackground(green_);
      toggleRead=false;
    }
    if (toggleSetSetting) {
      toggleSetSetting=false;
      toggleRead=true;
      payload = new ArrayList<Character>();
      payload.add(char( round(confSelectSetting.value())) );
      sendRequestMSP(requestMSP(MSP_SELECT_SETTING,payload.toArray( new Character[payload.size()]) )); 
    }
    if (toggleCalibAcc) {
      toggleCalibAcc=false;
      sendRequestMSP(requestMSP(MSP_ACC_CALIBRATION));
    }
    if (toggleCalibMag) {
      toggleCalibMag=false;
      sendRequestMSP(requestMSP(MSP_MAG_CALIBRATION));
    }
         
    //******************************************************************************************
    if ((toggleWriteServo || toggleWriteServoLive )&& !toggleRead) {   // MSP_SET_SERVO_CONF
      toggleWriteServo=false;
      payload = new ArrayList<Character>();
      if (multiType == AIRPLANE || multiType == PPM_TO_SERVO || multiType == HELI_120_CCPM || multiType == HELI_90_DEG){
        for( i=0;i<8;i++){
          int q= (int)ServoSliderMIN[i].value(); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Min
          q= (int)ServoSliderMAX[i].value(); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Max
          q= (int)(ServoSliderC[i].value()); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Servo centers
  
          servoRATE[i] = int(RateSlider[i].value());
          if ((int)Bbox.getArrayValue()[i]==1){ servoRATE[i] = abs(servoRATE[i]);}else{ servoRATE[i] = abs(servoRATE[i])*-1;}// Direction
  
          if(i==5 && ( multiType == HELI_120_CCPM || multiType == HELI_90_DEG) )servoRATE[5] =(int)Bbox.getArrayValue()[5]; // Yaw servo Direction
          if( multiType == HELI_120_CCPM ){
            if(i==3  ){servoRATE[3] = (int)Mbox.getArrayValue()[2]+(int)Mbox.getArrayValue()[1]*2+(int)Mbox.getArrayValue()[0]*4;}
            if(i==4  ){servoRATE[4] = (int)Mbox.getArrayValue()[5]+(int)Mbox.getArrayValue()[4]*2+(int)Mbox.getArrayValue()[3]*4;}
            if(i==6  ){servoRATE[6] = (int)Mbox.getArrayValue()[8]+(int)Mbox.getArrayValue()[7]*2+(int)Mbox.getArrayValue()[6]*4;}
            //bbb
          }
          payload.add(char(servoRATE[i])); // servoRATE
        }
      }
       //******************************************************************************************
      else{
        for( i=0;i<8;i++){ servoRATE[i]=round(RateSlider[i].value()); payload.add(char(servoRATE[i]));}// servoRATE
        for( i=0;i<8;i++) { int q= (int)(ServoSliderC[i].value())+512; payload.add(char (q % 256) ); payload.add(char (q / 256)  ); } // Servo centers...
        for( i=0;i<8;i++){ if ((int)Bbox.getArrayValue()[i]==1){ payload.add(char(11)); }else{ payload.add(char(9));} } // Direction
      }
      sendRequestMSP(requestMSP(MSP_SET_SERVO_CONF,payload.toArray( new Character[payload.size()]) ));
      toggleWriteServoLive=false;      
      toggleWaitHeli=true;
    }
    //**************************************************************************************** 
    if (toggleWriteWing || toggleWriteWingLive){ // MSP_SET_SERVO_CONF
      toggleWriteWing=false; toggleWriteWingLive=false;
      if (multiType == TRI || multiType == FLYING_WING || multiType == BI || multiType == DUALCOPTER  || multiType == SINGLECOPTER) { // TRI & Flying Wing
        int nBoxes;
        payload = new ArrayList<Character>();
        if (multiType == TRI) {nBoxes = 1;}else{nBoxes = 4;}
        
        if (multiType == FLYING_WING){
          servoRATE[3] = (int)Wbox.getArrayValue()[2]+(int)Wbox.getArrayValue()[0]*2;
          servoRATE[4] = (int)Wbox.getArrayValue()[3]+(int)Wbox.getArrayValue()[1]*2;
          RateSlider[3].setValue((int)servoRATE[3]);
          RateSlider[4].setValue((int)servoRATE[4]);
        }
        if (multiType == SINGLECOPTER){
          servoRATE[3] = (int)Wbox.getArrayValue()[0]+(int)Wbox.getArrayValue()[1]*2;
          servoRATE[4] = (int)Wbox.getArrayValue()[2]+(int)Wbox.getArrayValue()[3]*2;
          servoRATE[5] = (int)Wbox.getArrayValue()[4]+(int)Wbox.getArrayValue()[5]*2;
          servoRATE[6] = (int)Wbox.getArrayValue()[6]+(int)Wbox.getArrayValue()[7]*2;
          RateSlider[3].setValue((int)servoRATE[3]);
          RateSlider[4].setValue((int)servoRATE[4]);
          RateSlider[5].setValue((int)servoRATE[5]);
          RateSlider[6].setValue((int)servoRATE[6]);
        }
        if (multiType == DUALCOPTER){
          servoRATE[4] = (int)Wbox.getArrayValue()[0];
          servoRATE[5] = (int)Wbox.getArrayValue()[1];
          RateSlider[4].setValue((int)servoRATE[4]);
          RateSlider[5].setValue((int)servoRATE[5]);
        }
        if(multiType == TRI){
          RateSlider[5].setValue((int)Wbox.getArrayValue()[0]);
        }
         if (multiType == BI){           
          servoRATE[4] = (int)Wbox.getArrayValue()[2]+(int)Wbox.getArrayValue()[0]*2; // L servo
          servoRATE[5] = (int)Wbox.getArrayValue()[3]+(int)Wbox.getArrayValue()[1]*2; // R servo          
          RateSlider[4].setValue((int)servoRATE[4]);
          RateSlider[5].setValue((int)servoRATE[5]);
        }
        
        for( i=0;i<8;i++){
          int q= (int)ServoSliderMIN[i].value(); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Min
          q= (int)ServoSliderMAX[i].value(); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Max
          q= (int)(ServoSliderC[i].value()); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Servo centers

          servoRATE[i] = int(RateSlider[i].value());
          
          payload.add(char(servoRATE[i])); // servoRATE
        }
        sendRequestMSP(requestMSP(MSP_SET_SERVO_CONF,payload.toArray( new Character[payload.size()]) )); // Send settings
      }
      if(toggleGimbal || toggleTrigger){  // MSP_SET_SERVO_CONF
        payload = new ArrayList<Character>();
     
        ServoMIN[0]=(int)GimbalSlider[0].value();  ServoMIN[1]=(int)GimbalSlider[4].value(); ServoMIN[2]=(int)GimbalSlider[8].value();
        ServoMAX[0]=(int)GimbalSlider[1].value();  ServoMAX[1]=(int)GimbalSlider[5].value(); ServoMAX[2]=(int)GimbalSlider[9].value();
        ServoMID[0]=(int)GimbalSlider[2].value();  ServoMID[1]=(int)GimbalSlider[6].value(); ServoMID[2]=(int)GimbalSlider[10].value();
        servoRATE[0]=(int)GimbalSlider[3].value();servoRATE[1]=(int)GimbalSlider[7].value();servoRATE[2]=(int)GimbalSlider[11].value();
        
        for( i=0;i<8;i++) {
          int q;
          q= (int)ServoMIN[i]; payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Min
          q= (int)ServoMAX[i]; payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Max
          q= (int)ServoMID[i]; payload.add(char (q % 256) ); payload.add(char (q / 256)  ); // Servo centers
          payload.add(char(servoRATE[i])); // servoRATE
        }
        sendRequestMSP(requestMSP(MSP_SET_SERVO_CONF,payload.toArray( new Character[payload.size()]) )); // Send settings    
    }
  }

  if(gimbal && gimbalConfig ){
       if((int)GimbalSlider[11].value()==1) { GimbalSlider[11].setCaptionLabel("Trig_Reversed"); }else{ GimbalSlider[11].setCaptionLabel("Trig_Normal");}
     int ValLow;
    for( i=2;i<11;i+=4){
      if(i > 8){ValLow=1001;}else{ValLow=1201;}
      if (GimbalSlider[i].value() < ValLow ) {GimbalSlider[i].setMin(0).setMax(12);}
      switch((int)GimbalSlider[i].value()) {
      case 0: GimbalSlider[i].setCaptionLabel("ROLL") ;break;
      case 1: GimbalSlider[i].setCaptionLabel("NICK") ;break;
      case 2: GimbalSlider[i].setCaptionLabel("YAW")  ;break;
      case 3: GimbalSlider[i].setCaptionLabel("THRO") ;break;
      case 4: GimbalSlider[i].setCaptionLabel("AUX1") ;break;
      case 5: GimbalSlider[i].setCaptionLabel("AUX2") ;break;
      case 6: GimbalSlider[i].setCaptionLabel("AUX3") ;break;
      case 7: GimbalSlider[i].setCaptionLabel("AUX4") ;break;
      case 8: GimbalSlider[i].setCaptionLabel("AUX5") ;break;
      case 9: GimbalSlider[i].setCaptionLabel("AUX6") ;break;
      case 10:GimbalSlider[i].setCaptionLabel("AUX7") ;break;
      case 11:GimbalSlider[i].setCaptionLabel("AUX8") ;break;
      default:GimbalSlider[i].setCaptionLabel("MID") ;
      }
      if (GimbalSlider[i].value() == 12 ){
        GimbalSlider[i].setMin(ValLow-1).setMax(2000);
        if (i==10){GimbalSlider[i].setMin(ValLow-1).setMax(30000);}
        GimbalSlider[i].setValue(ValLow+1);
      }
    }
  }  
  
 if(ServosActive){
     int BreakPoint =Centerlimits[0]+1;
    for( i=0;i<8;i++){
      if (ServoSliderC[i].value() < BreakPoint ) {ServoSliderC[i].setMin(0).setMax(12);}
      switch((int)ServoSliderC[i].value()) {
      case 0: ServoSliderC[i].setCaptionLabel("ROLL") ;break;
      case 1: ServoSliderC[i].setCaptionLabel("NICK") ;break;
      case 2: ServoSliderC[i].setCaptionLabel("YAW")  ;break;
      case 3: ServoSliderC[i].setCaptionLabel("THRO") ;break;
      case 4: ServoSliderC[i].setCaptionLabel("AUX1") ;break;
      case 5: ServoSliderC[i].setCaptionLabel("AUX2") ;break;
      case 6: ServoSliderC[i].setCaptionLabel("AUX3") ;break;
      case 7: ServoSliderC[i].setCaptionLabel("AUX4") ;break;
      case 8: ServoSliderC[i].setCaptionLabel("AUX5") ;break;
      case 9: ServoSliderC[i].setCaptionLabel("AUX6") ;break;
      case 10:ServoSliderC[i].setCaptionLabel("AUX7") ;break;
      case 11:ServoSliderC[i].setCaptionLabel("AUX8") ;break;
      default:ServoSliderC[i].setCaptionLabel("MID") ;
      }
      if (ServoSliderC[i].value() == 12 ){
        ServoSliderC[i].setMin(BreakPoint-1).setMax(Centerlimits[1]);
        ServoSliderC[i].setValue(BreakPoint+1);
      }
    }
  }

    
   if (toggleWing || toggleServo ||  toggleGimbal || toggleTrigger){
      buttonLIVE.show();
      SaveSERVO.show();
      ServosActive=true;
    } else{
      buttonLIVE.hide();
      SaveSERVO.hide();
      buttonExport.hide();
      toggleLive=false;
      ServosActive=false;}
    
   //******************************************************************************************
    
    if (toggleWrite) {
      toggleWrite=false;
      
      // MSP_SET_RC_TUNING
      payload = new ArrayList<Character>();
      payload.add(char( round(confRC_RATE.value()*100)) );
      payload.add(char( round(confRC_EXPO.value()*100)) );
      payload.add(char( round(rollPitchRate.value()*100)) );
      payload.add(char( round(yawRate.value()*100)) );
      payload.add(char( round(dynamic_THR_PID.value()*100)) );
      payload.add(char( round(throttle_MID.value()*100)) );
      payload.add(char( round(throttle_EXPO.value()*100)) );
      sendRequestMSP(requestMSP(MSP_SET_RC_TUNING,payload.toArray( new Character[payload.size()]) ));

      // MSP_SET_PID
      payload = new ArrayList<Character>();
      for(i=0;i<PIDITEMS;i++) {
        byteP[i] = (round(confP[i].value()*10));
        byteI[i] = (round(confI[i].value()*1000));
        byteD[i] = (round(confD[i].value()));
      }

      //POS-4 POSR-5 NAVR-6 use different dividers
      byteP[4] = (round(confP[4].value()*100.0));
      byteI[4] = (round(confI[4].value()*100.0));

      byteP[5] = (round(confP[5].value()*10.0));
      byteI[5] = (round(confI[5].value()*100.0));
      byteD[5] = (round(confD[5].value()*10000.0))/10;

      byteP[6] = (round(confP[6].value()*10.0));
      byteI[6] = (round(confI[6].value()*100.0));
      byteD[6] = (round(confD[6].value()*10000.0))/10;

      for(i=0;i<PIDITEMS;i++) {
          payload.add(char(byteP[i]));  
          payload.add(char(byteI[i]));  
          payload.add(char(byteD[i])); 
      }
      sendRequestMSP(requestMSP(MSP_SET_PID,payload.toArray(new Character[payload.size()])));

      // MSP_SET_BOX
      payload = new ArrayList<Character>();
      for(i=0;i<CHECKBOXITEMS;i++) {
        activation[i] = 0;
        for(aa=0;aa<12;aa++) {
          activation[i] += (int)(checkbox[i].arrayValue()[aa]*(1<<aa));
          //MWI.setProperty("box."+i+".aux"+i/3+"."+(aa%3),String.valueOf(checkbox[i].arrayValue()[aa]*(1<<aa)));
        }
        payload.add(char (activation[i] % 256) ); payload.add(char (activation[i] / 256)  );
      }
      sendRequestMSP(requestMSP(MSP_SET_BOX,payload.toArray(new Character[payload.size()])));
     
      
      // MSP_SET_MISC
      payload = new ArrayList<Character>();
      
      intPowerTrigger = (round(confPowerTrigger.value()));
      payload.add(char(intPowerTrigger % 256)); payload.add(char(intPowerTrigger / 256)); //a

      
      // ThrVal   minthrottle,maxthrottle,mincommand,FSthrottle   b,c,d,e
      for( i=0;i<4;i++) {int q= (int)(confINF[i].value()); payload.add(char (q % 256) ); payload.add(char (q / 256)  ); }
      
      // PermanentLog
      int nn= round(confINF[4].value()*10); payload.add(char (nn - ((nn>>8)<<8) )); payload.add(char (nn>>8));// f
     
       
       nn= round(confINF[5].value());
       payload.add(char (nn - ((nn>>8)<<8)));   payload.add(char (nn>>8)); // g 32b
       payload.add(char (nn - ((nn>>16)<<16))); payload.add(char (nn>>16));
      
      
      // MagDec
      nn= round(confINF[6].value()*10); payload.add(char (nn - ((nn>>8)<<8) )); payload.add(char (nn>>8)); // h

      
      // VBatscale
      nn= round(VBat[0].value()); payload.add(char (nn)); // i
      for( i=1;i<4;i++) { int q= int(VBat[i].value()*10); payload.add(char (q)); } // j,k,l
      
      sendRequestMSP(requestMSP(MSP_SET_MISC,payload.toArray(new Character[payload.size()])));
      
          
      // MSP_EEPROM_WRITE
      sendRequestMSP(requestMSP(MSP_EEPROM_WRITE));
      
      updateModel(); // update model with view value
    }

    if (toggleRXbind) {
      toggleRXbind=false;
      sendRequestMSP(requestMSP(MSP_BIND));
      bSTOP();
      InitSerial(9999);
    }

    while (g_serial.available()>0) {
      c = (g_serial.read());

      if (c_state == IDLE) {
        c_state = (c=='$') ? HEADER_START : IDLE;
      } else if (c_state == HEADER_START) {
        c_state = (c=='M') ? HEADER_M : IDLE;
      } else if (c_state == HEADER_M) {
        if (c == '>') {
          c_state = HEADER_ARROW;
        } else if (c == '!') {
          c_state = HEADER_ERR;
        } else {
          c_state = IDLE;
        }
      } else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
        /* is this an error message? */
        err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
        dataSize = (c&0xFF);
        /* reset index variables */
        p = 0;
        offset = 0;
        checksum = 0;
        checksum ^= (c&0xFF);
        /* the command is to follow */
        c_state = HEADER_SIZE;
      } else if (c_state == HEADER_SIZE) {
        cmd = (byte)(c&0xFF);
        checksum ^= (c&0xFF);
        c_state = HEADER_CMD;
      } else if (c_state == HEADER_CMD && offset < dataSize) {
          checksum ^= (c&0xFF);
          inBuf[offset++] = (byte)(c&0xFF);
      } else if (c_state == HEADER_CMD && offset >= dataSize) {
        /* compare calculated and transferred checksum */
        if ((checksum&0xFF) == (c&0xFF)) {
          if (err_rcvd) {
            //System.err.println("Copter did not understand request type "+c);
          } else {
            /* we got a valid response packet, evaluate it */
            evaluateCommand(cmd, (int)dataSize);
          }
        } else {
          System.out.println("invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
          System.out.print("<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");
          for (i=0; i<dataSize; i++) {
            if (i!=0) { System.err.print(' '); }
            System.out.print((inBuf[i] & 0xFF));
          }
          System.out.println("} ["+c+"]");
          System.out.println(new String(inBuf, 0, dataSize));
        }
        c_state = IDLE;
      }
    }
  }

  background(80);
  // version Background
  fill(40, 40, 40);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  rect(5,5+tabHeight,113,40+tabHeight);
  textFont(font15);
  // version
  fill(255, 255, 255);
  text("multiwii.com",16,19+tabHeight);
  text("V",16,35+tabHeight);
  text(version, 27, 35+tabHeight);
  text(i2cError,xGraph+410,yGraph-10);
  text(cycleTime,xGraph+290,yGraph-10);

  // ---------------------------------------------------------------------------------------------
  // GPS DATA
  // ---------------------------------------------------------------------------------------------
  // GPS Background
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  rect(xGPS-5,yGPS-15,xGPS+140,yGPS+95);
  // GPS DATA
  fill(255, 255, 255);
  text("GPS",xGPS,yGPS);
  text(GPS_altitude,xGPS+50,yGPS+15);
  text(GPS_latitude,xGPS+50,yGPS+30);
  text(GPS_longitude,xGPS+50,yGPS+45);
  text(GPS_speed,xGPS+50,yGPS+60);
  text(GPS_numSat,xGPS+50,yGPS+75);
  text(GPS_distanceToHome,xGPS+65,yGPS+90);
  textFont(font12);
  text("alt   :",xGPS,yGPS+15);
  text("lat   :",xGPS,yGPS+30);
  text("lon   :",xGPS,yGPS+45);
  text("speed :",xGPS,yGPS+60);
  text("sat   :",xGPS,yGPS+75);
  text("dist home : ",xGPS,yGPS+90);
  // ---------------------------------------------------------------------------------------------

  text("I2C error:",xGraph+350,yGraph-10);
  text("Cycle Time:",xGraph+220,yGraph-10);
  
  text("Power", xGraph -5, yGraph -65);
  textFont(font9);
  
  text("   Voltage:", xGraph -5, yGraph -55);
  text(bytevbat/10.0 + " V", xGraph +50, yGraph -55);
  
  text("   Current:", xGraph -5, yGraph -45);
  text(amperage/10.0 + " A", xGraph +50, yGraph -45);
  
  text("   Total:", xGraph -5, yGraph -35);
  text(pMeterSum * 1.0 + " mAh", xGraph +50, yGraph -35);

  fill(255,255,255);

  axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
  altSlider.setValue(alt/100);headSlider.setValue(head);magxSlider.setValue(magx);magySlider.setValue(magy);magzSlider.setValue(magz);
  debug1Slider.setValue(debug1);debug2Slider.setValue(debug2);debug3Slider.setValue(debug3);debug4Slider.setValue(debug4);
  
   
  for(i=0;i<8;i++) { //Hide all sliders
    motSlider[i].setValue(mot[i]).hide();
    servoSliderH[i].setValue(servo[i]).hide();
    servoSliderV[i].setValue(servo[i]).hide();
  }  
  
  if(gimbalConfig){
    if (toggleGimbal == true){
      TxtInfo3.show();
      for( i=0;i<8;i++) GimbalSlider[i].show();
    }else{
     TxtInfo3.hide();
     for( i=0;i<8;i++) GimbalSlider[i].hide();
    }
    if(toggleTrigger){
      TxtInfo4.show();
      for( i=8;i<12;i++) GimbalSlider[i].show();
    }else{
     TxtInfo4.hide();
     for( i=8;i<12;i++)  GimbalSlider[i].hide();
    }  
  }

  if (multiType == FLYING_WING  || multiType == TRI || multiType == BI  || multiType == DUALCOPTER  || multiType == SINGLECOPTER) { //|| multiType == DUALCOPTER
    buttonWing.show();
    TxtInfo.show();
    for (i=0;i<8;i++) {
      motSlider[i].setValue(mot[i]).hide();
      servoSliderH[i].setValue(servo[i]).hide(); 
      servoSliderV[i].setValue(servo[i]).hide();
    }
  if (toggleWing==true) {
    if(multiType != SINGLECOPTER) {TxtLeftW.show();  TxtRightW.show();} else{ TxtMin.show();TxtMax.show();TxtMids.show();}
    TxtRevW.show();TxtRevR.show(); Wbox.show();  SaveWing.show();
    if(multiType == TRI){
      ServoSliderC[5]  .show();ServoSliderMIN[5].show();ServoSliderMAX[5].show();
    }
  } else {
    if(multiType != SINGLECOPTER) {TxtLeftW.hide(); TxtRightW.hide();}else{ TxtMin.hide();TxtMax.hide();TxtMids.hide();}
    TxtRevW.hide();TxtRevR.hide(); Wbox.hide(); SaveWing.hide();buttonWing.setLabel(" Servo");
    for (i=0;i<8;i++) {
      ServoSliderC[i]  .hide();
      ServoSliderMIN[i].hide();
      ServoSliderMAX[i].hide();
    }
  }
}
    
  if (multiType == AIRPLANE || multiType == HELI_120_CCPM || multiType == PPM_TO_SERVO  || multiType == HELI_90_DEG ){
    TxtInfo.show();
    buttonSERVO.show(); 
  if ( multiType == HELI_120_CCPM ){
    if (toggleServo==true ){
      for (i=0;i<6;i++) TxtMIX[i].show();Mbox.show();
    }else{
      for (i=0;i<6;i++) TxtMIX[i].hide();Mbox.hide();
    }
  }
    for (i=0;i<8;i++) {
      motSlider[i].setValue(mot[i]).hide();
      servoSliderH[i].setValue(servo[i]).hide(); 
      servoSliderV[i].setValue(servo[i]).hide();
      ServoSliderC[i].hide();
      if (toggleServo==true ) {
        SaveSERVO.show();
        Bbox.show();
        BtServo[i].show();
        
        if(i<1 && flapperons){ServoSliderMIN[i].show();ServoSliderMAX[i].show();}
        if(i==2 && flaps && toggleServo){ServoSliderMIN[i].show();ServoSliderMAX[i].show();TxtAux.show();ServoSliderC[i].show();}//
        if(i<5 && flaps) BtAUX[i].show();
        RateSlider[i].show();
        TxtRates.show();
        TxtMids.show();
        TxtRev.show();
        checkboxRev[i].show();
        if(i>2)ServoSliderC[i].show();
        buttonSERVO.setLabel("SERVO");
        
        if( multiType == PPM_TO_SERVO){ServoSliderC[i].show();ServoSliderMIN[i].show();ServoSliderMAX[i].show();RateSlider[i].hide();}
        
        if( multiType == HELI_120_CCPM){
          if(i>2) {ServoSliderMIN[i].show();ServoSliderMAX[i].show();TxtMin.show();TxtMax.show();}
        }
        if( multiType == HELI_90_DEG ){
          if(i<3) {
            ServoSliderC[i].show();ServoSliderMIN[i].show();
            ServoSliderMAX[i].show();TxtMin.show();TxtMax.show();
          }
        ServoSliderMIN[5].show(); ServoSliderMAX[5].show();
        }
        
      } else {
        if(i<5)BtAUX[i].hide();
        if(flaps)TxtAux.hide();
        
        Bbox.hide();
//        SaveSERVO.hide();
        TxtRates.hide();
        TxtMids.hide();
        TxtRev.hide();
        RateSlider[i].hide();
        ServoSliderC[i].hide();
        BtServo[i].hide();
        checkboxRev[i].hide();
        ServoSliderMIN[i].hide();ServoSliderMAX[i].hide(); 
        buttonSERVO.setLabel("SERVO");
        if ( multiType == HELI_120_CCPM ||  multiType == HELI_90_DEG){TxtMin.hide();TxtMax.hide();
        }
      }
    }
  }

  
  stroke(255);
  a=radians(angx);
  if (angy<-90) {b=radians(-180 - angy);}
  else if (angy>90) {b=radians(+180 - angy);}
  else{ b=radians(angy);}
  h=radians(head);

  // ---------------------------------------------------------------------------------------------
  // DRAW MULTICOPTER TYPE
  // ---------------------------------------------------------------------------------------------
  float size = 38.0;
  // object
  fill(255,255,255);
  pushMatrix();
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+30,yObj/2-40,0,0,1,0);
  translate(xObj,yObj);
  directionalLight(200,200,200, 0, 0, -1);
  rotateZ(h);rotateX(b);rotateY(a);
  stroke(150,255,150);
  strokeWeight(0);sphere(size/3);strokeWeight(3);
  line(0,0, 10,0,-size-5,10);line(0,-size-5,10,+size/4,-size/2,10); line(0,-size-5,10,-size/4,-size/2,10);
  stroke(255);
  int MotToggleMove=200;

  textFont(font12);
   if (toggleGimbal || toggleTrigger) { //if (multiType == GIMBAL)
    noLights();text("GIMBAL", -20,-55);camera();popMatrix();
    text("GIMBAL", xMot,yMot+25);
 
    servoSliderH[1].setPosition(xMot,yMot+75).setCaptionLabel("ROLL") .show();
    servoSliderH[0].setPosition(xMot,yMot+35).setCaptionLabel("PITCH").show();
    if(camTrigger)servoSliderH[2].setPosition(xMot,yMot+120).setCaptionLabel("Trigg").show();
    
  } else if (multiType == TRI) { //TRI
    drawMotor(    0, +size, byteMP[0], 'L');
    drawMotor(+size, -size, byteMP[1], 'L');
    drawMotor(-size, -size, byteMP[2], 'R');
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();text(" TRICOPTER", -40,-50);camera();popMatrix();
    TxtRightW .hide();
    motSlider[0].setPosition(xMot+50,yMot+15).setHeight(100).setCaptionLabel("REAR").show();
    motSlider[1].setPosition(xMot+100,yMot-15).setHeight(100).setCaptionLabel("RIGHT").show();
    motSlider[2].setPosition(xMot,yMot-15).setHeight(100).setCaptionLabel("LEFT").show();
    if(InitServos ){
      InitServos=false;      
    }
    servoSliderH[5].setPosition(xMot,yMot+135).setCaptionLabel("SERVO " ).show();
    if(toggleWing){
      int Step=yServ+10;
      ServoSliderC[5]  .show().setPosition(xServ+100 ,Step+60);//.setCaptionLabel("Center" +yawServo);
      ServoSliderMIN[5].show().setPosition(xServ+100 ,Step+80).setCaptionLabel("Min");
      ServoSliderMAX[5].show().setPosition(xServ+180 ,Step+80).setCaptionLabel("Max");
    } 
    motToggle[0].setPosition(xMot+50-MotToggleMove,yMot+55).setCaptionLabel("REAR").show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot-15).setCaptionLabel("RIGHT").show();
    motToggle[2].setPosition(xMot-MotToggleMove,yMot-15).setCaptionLabel("LEFT").show();
  } else if (multiType == QUADP) { //QUAD+
    drawMotor(0,     +size, byteMP[0], 'R');
    drawMotor(+size, 0,     byteMP[1], 'L');
    drawMotor(-size, 0,     byteMP[2], 'L');
    drawMotor(0,     -size, byteMP[3], 'R');
    line(-size,0, +size,0);line(0,-size, 0,+size);
    noLights();text("QUADRICOPTER +", -40,-50);camera();popMatrix();
    
    motSlider[0].setPosition(xMot+50,yMot+75).setHeight(60).setCaptionLabel("REAR").show();
    motSlider[1].setPosition(xMot+100,yMot+35).setHeight(60).setCaptionLabel("RIGHT").show();
    motSlider[2].setPosition(xMot,yMot+35).setHeight(60).setCaptionLabel("LEFT").show();
    motSlider[3].setPosition(xMot+50,yMot-15).setHeight(60).setCaptionLabel("FRONT").show();
    
    motToggle[0].setPosition(xMot+50-MotToggleMove,yMot+75).show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot+35).show();
    motToggle[2].setPosition(xMot-MotToggleMove,yMot+35).show();
    motToggle[3].setPosition(xMot+50-MotToggleMove,yMot-15).show();
    
    
  } else if (multiType == QUADX) { //QUAD X
    drawMotor(+size, +size, byteMP[0], 'R');
    drawMotor(+size, -size, byteMP[1], 'L');
    drawMotor(-size, +size, byteMP[2], 'L');
    drawMotor(-size, -size, byteMP[3], 'R');
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(-size,+size, 0,0);line(+size,+size, 0,0);
    noLights();text("QUADRICOPTER X", -40,-50);camera();popMatrix();
    
    motSlider[0].setPosition(xMot+90,yMot+75).setHeight(60).setCaptionLabel("REAR_R") .show();
    motSlider[1].setPosition(xMot+90,yMot-15).setHeight(60).setCaptionLabel("FRONT_R").show();
    motSlider[2].setPosition(xMot+10,yMot+75).setHeight(60).setCaptionLabel("REAR_L") .show();
    motSlider[3].setPosition(xMot+10,yMot-15).setHeight(60).setCaptionLabel("FRONT_L").show(); 
    
    motToggle[0].setPosition(xMot+90-MotToggleMove,yMot+75).show();
    motToggle[1].setPosition(xMot+90-MotToggleMove,yMot-15).show();
    motToggle[2].setPosition(xMot+10-MotToggleMove,yMot+75).show();
    motToggle[3].setPosition(xMot+10-MotToggleMove,yMot-15).show(); 
  } else if (multiType == BI) { //BI
    drawMotor(-size, 0, byteMP[0], 'R');
    drawMotor(+size, 0, byteMP[1], 'L');
    line(0-size,0, 0,0);  line(0+size,0, 0,0);line(0,size*1.5, 0,0);
    noLights();text("BICOPTER", -30,-20);camera();popMatrix();
   
    motSlider[0].setPosition(xMot,yMot+30).setHeight(55).setCaptionLabel("").show();
    motSlider[1].setPosition(xMot+100,yMot+30).setHeight(55).setCaptionLabel("").show();
    servoSliderH[4].setPosition(xMot,yMot+100).setWidth(60).setCaptionLabel("").show();
    servoSliderH[5].setPosition(xMot+80,yMot+100).setWidth(60).setCaptionLabel("").show();
    
    motToggle[0].setPosition(xMot-MotToggleMove,yMot+30).setCaptionLabel("").show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot+30).setCaptionLabel("").show();

    if(toggleWing){
      int Step=yServ+10;
      int ServoN =4;
      ServoSliderC[ServoN]  .setPosition(xServ+100 ,Step+0+60).show().setCaptionLabel("  Center");
      ServoSliderMIN[ServoN].show().setCaptionLabel("  Min") .setPosition(xServ+100 ,Step+0+80);
      ServoSliderMAX[ServoN].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+0+80);
      Step+=20;
      ServoN =5;
      ServoSliderC[ServoN]    .setPosition(xServ+100 ,Step+80+60).show().setCaptionLabel("  Center");
      ServoSliderMIN[ServoN].show().setCaptionLabel("  Min") .setPosition(xServ+100 ,Step+80+80);
      ServoSliderMAX[ServoN].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+80+80);
    }
    
  }else if (multiType == Y6) { //Y6
    drawMotor(       +7+0,   +7+size, byteMP[0], 'L');
    drawMotor(    +7+size,   +7-size, byteMP[1], 'R');
    drawMotor(    +7-size,   +7-size, byteMP[2], 'R');
    translate(0,0,-7);
    drawMotor(       -7+0,   -7+size, byteMP[3], 'R');
    drawMotor(    -7+size,   -7-size, byteMP[4], 'L');
    drawMotor(    -7-size,   -7-size, byteMP[5], 'L');
    line(-size,-size,0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();text("TRICOPTER Y6", -40,-55);camera();popMatrix();

    motSlider[0].setPosition(xMot+50,yMot+23).setHeight(50).setCaptionLabel("REAR").show();
    motSlider[1].setPosition(xMot+100,yMot-18).setHeight(50).setCaptionLabel("RIGHT").show();
    motSlider[2].setPosition(xMot,yMot-18).setHeight(50).setCaptionLabel("LEFT").show();
    motSlider[3].setPosition(xMot+50,yMot+87).setHeight(50).setCaptionLabel("U_REAR").show();
    motSlider[4].setPosition(xMot+100,yMot+48).setHeight(50).setCaptionLabel("U_RIGHT").show();
    motSlider[5].setPosition(xMot,yMot+48).setHeight(50).setCaptionLabel("U_LEFT").show();
    
    motToggle[0].setPosition(xMot+50-MotToggleMove,yMot+48).show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot-18).show();
    motToggle[2].setPosition(xMot-MotToggleMove,yMot-18).show();
    motToggle[3].setPosition(xMot+50-MotToggleMove,yMot+87).show();
    motToggle[4].setPosition(xMot+100-MotToggleMove,yMot+23).show();
    motToggle[5].setPosition(xMot-MotToggleMove,yMot+23).show();
  } else if (multiType == HEX6) { //HEX6
    drawMotor(+size, +0.55*size, byteMP[0], 'L');
    drawMotor(+size, -0.55*size, byteMP[1], 'R');
    drawMotor(-size, +0.55*size, byteMP[2], 'L');
    drawMotor(-size, -0.55*size, byteMP[3], 'R');
    drawMotor(    0,      -size, byteMP[4], 'L');
    drawMotor(    0,      +size, byteMP[5], 'R');
    line(-size,-0.55*size,0,0);line(size,-0.55*size,0,0);line(-size,+0.55*size,0,0);line(size,+0.55*size,0,0);line(0,+size,0,0);line(0,-size,0,0);
    noLights();text("HEXACOPTER", -40,-50);camera();popMatrix();

    motSlider[0].setPosition(xMot+90,yMot+65).setHeight(50).setCaptionLabel("REAR_R").show();
    motSlider[1].setPosition(xMot+90,yMot-5).setHeight(50) .setCaptionLabel("FRONT_R").show();
    motSlider[2].setPosition(xMot+5,yMot+65) .setHeight(50).setCaptionLabel("REAR_L").show();
    motSlider[3].setPosition(xMot+5,yMot-5 ) .setHeight(50).setCaptionLabel("FRONT_L").show(); 
    motSlider[4].setPosition(xMot+50,yMot-20).setHeight(50).setCaptionLabel("FRONT").show(); 
    motSlider[5].setPosition(xMot+50,yMot+90).setHeight(50).setCaptionLabel("REAR").show(); 
    
    motToggle[0].setPosition(xMot+90-MotToggleMove,yMot+65).show();
    motToggle[1].setPosition(xMot+90-MotToggleMove,yMot-5) .show();
    motToggle[2].setPosition(xMot+5-MotToggleMove,yMot+65) .show();
    motToggle[3].setPosition(xMot+5-MotToggleMove,yMot-5 ) .show(); 
    motToggle[4].setPosition(xMot+50-MotToggleMove,yMot-20).show(); 
    motToggle[5].setPosition(xMot+50-MotToggleMove,yMot+90).show();
  } else if (multiType == FLYING_WING) { //FLYING_WING
    line(0,0, 1.8*size,size);line(1.8*size,size,1.8*size,size-30);  line(1.8*size,size-30,0,-1.5*size);
    line(0,0, -1.8*size,+size);line(-1.8*size,size,-1.8*size,+size-30);    line(-1.8*size,size-30,0,-1.5*size);
    noLights();text("FLYING WING", -40,-50);camera();popMatrix();

    servoSliderV[3].setPosition(xMot+5,yMot+10).setCaptionLabel("LEFT").show(); 
    servoSliderV[4].setPosition(xMot+100,yMot+10).setCaptionLabel("RIGHT").show();
    motSlider[0].setPosition(xMot+50,yMot+30).setHeight(90).setCaptionLabel("Mot").show();
    TX_StickSlider[RCPitch].setCaptionLabel("Elev");
    TX_StickSlider[RCYaw ].setCaptionLabel("Rudd");
	
    if(toggleWing){
      int Step=yServ;
      ServoSliderC[3].show().setPosition(xServ+100 ,Step+0+60);//.setCaptionLabel("  Center");
      ServoSliderMIN[3].show().setCaptionLabel("  Min") .setPosition(xServ+100  ,Step+0+80);
      ServoSliderMAX[3].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+0+80);
      Step+=10;
      ServoSliderC[4].show().setPosition(xServ+100 ,Step+80+60);// .show().setCaptionLabel("  Center");
      ServoSliderMIN[4].show().setCaptionLabel("  Min") .setPosition(xServ+100  ,Step+80+80);
      ServoSliderMAX[4].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+80+80);
    }
	  
  } else if (multiType == Y4) { //Y4
    drawMotor(       +15+0,      +size, byteMP[0], 'R');
    drawMotor(       +size,      -size, byteMP[1], 'L');
    drawMotor(       -size,      -size, byteMP[3], 'R');
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    translate(0,0,-7);
    drawMotor(       -15+0,      +size, byteMP[2], 'L');
    noLights();text("Y4", -5,-50);camera();popMatrix();
    
    motSlider[0].setPosition(xMot+80,yMot+75).setHeight(60).setCaptionLabel("REAR_1").show();
    motSlider[1].setPosition(xMot+90,yMot-15).setHeight(60).setCaptionLabel("FRONT_R").show();
    motSlider[2].setPosition(xMot+30,yMot+75).setHeight(60).setCaptionLabel("REAR_2").show();
    motSlider[3].setPosition(xMot+10,yMot-15).setHeight(60).setCaptionLabel("FRONT_L").show(); 
        
    motToggle[0].setPosition(xMot+70-MotToggleMove,yMot+75).show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot-15).show();
    motToggle[2].setPosition(xMot+40-MotToggleMove,yMot+75).show();
    motToggle[3].setPosition(xMot+10-MotToggleMove,yMot-15).show(); 

  } else if (multiType == 18) { //HEX6 H
    drawMotor(+size, +1.2*size, byteMP[0], 'R');
    drawMotor(+size, -1.2*size, byteMP[1], 'L');
    drawMotor(-size, +1.2*size, byteMP[2], 'L');
    drawMotor(-size, -1.2*size, byteMP[3], 'R');
    drawMotor(-size,         0, byteMP[4], 'L');
    drawMotor(+size,         0, byteMP[5], 'R');

    line(+size, +1.2*size,+size, -1.2*size);line(-size, +1.2*size,-size, -1.2*size);
    line(+size,0,0,0);  line(-size,0,0,0);
    noLights();text("HEXACOPTER H", -45,-60);camera();popMatrix();

    motSlider[0].setPosition(xMot+80,yMot+90).setHeight(45).setCaptionLabel("REAR_R").show();
    motSlider[1].setPosition(xMot+80,yMot-20).setHeight(45).setCaptionLabel("FRONT_R").show();
    motSlider[2].setPosition(xMot+25,yMot+90).setHeight(45).setCaptionLabel("REAR_L").show();
    motSlider[3].setPosition(xMot+25,yMot-20).setHeight(45).setCaptionLabel("FRONT_L").show();
    motSlider[4].setPosition(xMot+90,yMot+35).setHeight(45).setCaptionLabel("RIGHT").show();
    motSlider[5].setPosition(xMot+5,yMot+35) .setHeight(45).setCaptionLabel("LEFT").show();
    
    motToggle[0].setPosition(xMot+90-MotToggleMove,yMot+90).show();
    motToggle[1].setPosition(xMot+90-MotToggleMove,yMot-20).show();
    motToggle[2].setPosition(xMot+ 5-MotToggleMove,yMot+90).show();
    motToggle[3].setPosition(xMot+ 5-MotToggleMove,yMot-20).show();
    motToggle[4].setPosition(xMot+90-MotToggleMove,yMot+35).show();
    motToggle[5].setPosition(xMot+5 -MotToggleMove,yMot+35).show();
  } else if (multiType == HEX6X) { //HEX6 X
    drawMotor(+0.55*size, +size, byteMP[0], 'L');
    drawMotor(+0.55*size, -size, byteMP[1], 'L');
    drawMotor(-0.55*size, +size, byteMP[2], 'R');
    drawMotor(-0.55*size, -size, byteMP[3], 'R');
    drawMotor(     +size, 0,     byteMP[4], 'R');
    drawMotor(     -size, 0,     byteMP[5], 'L');

    line(-0.55*size,-size,0,0);line(-0.55*size,size,0,0);line(+0.55*size,-size,0,0);line(+0.55*size,size,0,0);line(+size,0,0,0);  line(-size,0,0,0);
    noLights();text("HEXACOPTER X", -45,-50);camera();popMatrix();

    motSlider[0].setPosition(xMot+80,yMot+90).setHeight(45).setCaptionLabel("REAR_R").show();
    motSlider[1].setPosition(xMot+80,yMot-20).setHeight(45).setCaptionLabel("FRONT_R").show();
    motSlider[2].setPosition(xMot+25,yMot+90).setHeight(45).setCaptionLabel("REAR_L").show();
    motSlider[3].setPosition(xMot+25,yMot-20).setHeight(45).setCaptionLabel("FRONT_L").show(); 
    motSlider[4].setPosition(xMot+90,yMot+35).setHeight(45).setCaptionLabel("RIGHT").show(); 
    motSlider[5].setPosition(xMot+5,yMot+35) .setHeight(45).setCaptionLabel("LEFT").show(); 
    
    motToggle[0].setPosition(xMot+80-MotToggleMove,yMot+90).show();
    motToggle[1].setPosition(xMot+80-MotToggleMove,yMot-20).show();
    motToggle[2].setPosition(xMot+25-MotToggleMove,yMot+90).show();
    motToggle[3].setPosition(xMot+25-MotToggleMove,yMot-20).show(); 
    motToggle[4].setPosition(xMot+110-MotToggleMove,yMot+35).show(); 
    motToggle[5].setPosition(xMot-5 -MotToggleMove,yMot+35).show();     
  } else if (multiType == OCTOX8 ) { //OCTOX8    
    motToggle[0].setPosition(xMot+110-MotToggleMove,yMot+65).show();
    motToggle[1].setPosition(xMot+90-MotToggleMove,yMot-45).show();
    motToggle[2].setPosition(xMot-10-MotToggleMove,yMot+65).show();
    motToggle[3].setPosition(xMot+10-MotToggleMove,yMot-45).show();
    motToggle[4].setPosition(xMot+90-MotToggleMove,yMot+95).show();
    motToggle[5].setPosition(xMot+110-MotToggleMove,yMot-15).show();
    motToggle[6].setPosition(xMot+10-MotToggleMove,yMot+95).show();
    motToggle[7].setPosition(xMot-10-MotToggleMove,yMot-15).show(); 
    
    noLights();text("OCTOCOPTER X", -45,-50);camera();popMatrix();
  } else if (multiType == OCTOFLATX) { //OCTOXP
    // GUI is the same for all 8 motor configs. multiType 12-13    
    motToggle[0].setPosition(xMot+10-MotToggleMove,yMot-15).show(); 
    motToggle[1].setPosition(xMot+90-MotToggleMove,yMot-15).show();
    motToggle[2].setPosition(xMot+90-MotToggleMove,yMot+75).show();
    motToggle[3].setPosition(xMot+10-MotToggleMove,yMot+75).show();
    motToggle[4].setPosition(xMot+50-MotToggleMove,yMot-35).show();
    motToggle[5].setPosition(xMot+110-MotToggleMove,yMot+35).show();
    motToggle[6].setPosition(xMot+50-MotToggleMove,yMot+95).show();
    motToggle[7].setPosition(xMot-10-MotToggleMove,yMot+35).show();
    
    noLights();text("OCTOCOPTER P", -45,-50);camera();popMatrix();
  } else if (multiType == OCTOFLATP) { //OCTOXX
    // GUI is the same for all 8 motor configs. multiType 12-13
        
    motToggle[0].setPosition(xMot+25-MotToggleMove,yMot-30) .show();//MIDFRONT_L
    motToggle[1].setPosition(xMot+110-MotToggleMove,yMot+5) .show();//FRONT_R
    motToggle[2].setPosition(xMot+75-MotToggleMove,yMot+85) .show();//MIDREAR_R
    motToggle[3].setPosition(xMot-10-MotToggleMove,yMot+55) .show();//REAR_L
    motToggle[4].setPosition(xMot-10-MotToggleMove,yMot+5)  .show();//FRONT_L
    motToggle[5].setPosition(xMot+75-MotToggleMove,yMot-30) .show();//MIDFRONT_R
    motToggle[6].setPosition(xMot+110-MotToggleMove,yMot+55).show();//REAR_R
    motToggle[7].setPosition(xMot+25-MotToggleMove,yMot+85) .show();//MIDREAR_L
    
    /*  Test with mororSliders on OCTO X
    motSlider[0].setPosition(xMot+25,yMot-30) .setCaptionLabel("").show();//MIDFRONT_L
    motSlider[1].setPosition(xMot+110,yMot+5) .setCaptionLabel("").show();//FRONT_R
    motSlider[2].setPosition(xMot+75,yMot+55) .setCaptionLabel("").show();//MIDREAR_R
    motSlider[3].setPosition(xMot-10,yMot+25) .setCaptionLabel("").show();//REAR_L
    motSlider[4].setPosition(xMot-10,yMot+5)  .setCaptionLabel("").show();//FRONT_L
    motSlider[5].setPosition(xMot+75,yMot-30) .setCaptionLabel("").show();//MIDFRONT_R
    motSlider[6].setPosition(xMot+110,yMot+25).setCaptionLabel("").show();//REAR_R
    motSlider[7].setPosition(xMot+25,yMot+55) .setCaptionLabel("").show();//MIDREAR_L
    */
    noLights();text("OCTOCOPTER X", -45,-50);camera();popMatrix();
  } else if (multiType == AIRPLANE) { //AIRPLANE
    float Span = size*1.3;  
    float VingRoot = Span*0.25;  
    // Wing
    line(0,0,  Span,0);   line(Span,0, Span, VingRoot);       line(Span, VingRoot, 0,VingRoot); 
    line(0,0,  -Span,0);   line(-Span,0, -Span, VingRoot);       line(-Span, VingRoot, 0,VingRoot);    
    // Stab
    line(-(size*0.4),size,  (size*0.4),size);   line(-(size*0.4),size+5,  (size*0.4),size+5); 
    line(-(size*0.4),size,  -(size*0.4),size+5);      line((size*0.4),size,  (size*0.4),size+5);     
    // Body  
    line(-2,size,  -2,-size+5); line(2,size,  2,-size+5); line( -2,-size+5,  2,-size+5);    
    // Fin 
    line(0,size-3,0,  0,size,15); line(0,size,15,  0,size+5,15);line(0,size+5,15,  0,size+5,0);       
    noLights();
    textFont(font12);
    text("AIRPLANE", -40,-50);camera();popMatrix();
  
    servoSliderH[3].setPosition(xMot,yMot-5)  .setCaptionLabel("Wing 1").show();
    servoSliderH[4].setPosition(xMot,yMot+25) .setCaptionLabel("Wing 2").show();
    servoSliderH[5].setPosition(xMot,yMot+55) .setCaptionLabel("Rudd").show();
    servoSliderH[6].setPosition(xMot,yMot+85) .setCaptionLabel("Elev").show();
    servoSliderH[7].setPosition(xMot,yMot+115).setCaptionLabel("Thro").show();
    TX_StickSlider[RCPitch].setCaptionLabel("Elev");
    TX_StickSlider[RCYaw ].setCaptionLabel("Rudd");
//    if(flapperons) { BtServo[0].setLabel("Flprn 1"); BtServo[1].setLabel("Flprn 2");}
    if(flaps)  { BtServo[2].setLabel("Flaps").setSize(60,12).setColorBackground(green_);servoSliderH[2].setPosition(xMot,yMot+130).setCaptionLabel("Flaps").show();}
    if( !flaps) {for(i=0;i<3;i++) BtServo[i].setLabel("").setSize(90,12).setColorBackground(black_);}//!flapperons &&
    
    BtServo[0].setLabel("").setSize(90,12).setColorBackground(black_);
    BtServo[1].setLabel("").setSize(90,12).setColorBackground(black_);
    BtServo[3].setLabel("Wing 1");
    BtServo[4].setLabel("Wing 2");
    BtServo[5].setLabel("Rudder");
    BtServo[6].setLabel("Elev");
    BtServo[7].setLabel("").setSize(90,12).setColorBackground(black_);//
    ServoSliderC[7].hide();ServoSliderMIN[7].hide();ServoSliderMAX[7].hide();
    RateSlider[0].hide(); RateSlider[1].hide(); RateSlider[7].hide(); RateSlider[2].hide();

  }else if (multiType == HELI_120_CCPM) { // 120 CCPM
    // HeliGraphics    
    float scalesize=size*0.8;
    // Rotor
    ellipse(0, 0, 2*scalesize, 2*scalesize);
    // Body  
    line(0,1.5*scalesize,-5,  -2,-0.5*scalesize,-5); line(0,1.5*scalesize,-5,  2,-0.5*scalesize,-5); line( -2,-0.5*scalesize,-5,  2,-0.5*scalesize,-5);    
    // Fin 
    float finpos = scalesize * 1.3;
    int HFin=0;
    int LFin=15;  
    line(0,finpos-3,0,  0,finpos+7,-LFin); line(0,finpos+7,-LFin,  0,finpos+10,-LFin);line(0,finpos+10,-LFin,  0,finpos+5,0); 
    line(0,finpos-3,0,  0,finpos,HFin); line(0,finpos,HFin,  0,finpos+5,HFin);line(0,finpos+5,HFin,  0,finpos+5,0); 
 
    // Stab
    line(-(scalesize*0.3),scalesize,-5,  (scalesize*0.3),scalesize,-5);   line(-(scalesize*0.3),scalesize+3,-5, (scalesize*0.3),scalesize+3,-5); 
    line(-(scalesize*0.3),scalesize,-5, -(scalesize*0.3),scalesize+3,-5); line((scalesize*0.3),scalesize,-5,    (scalesize*0.3),scalesize+3,-5);  
   
    noLights();
    textFont(font12);
    text("Heli 120 CCPM", -42,-50);camera();popMatrix();
        
    servoSliderV[7].setPosition(xMot,yMot)        .setCaptionLabel("Thro").show();
    servoSliderV[4].setPosition(xMot+40,yMot-15)  .setCaptionLabel("LEFT").show();
    servoSliderV[3].setPosition(xMot+70,yMot+10)  .setCaptionLabel("Nick").show();
    servoSliderH[5].setPosition(xMot+15,yMot+130) .setCaptionLabel("Yaw") .show();
    servoSliderV[6].setPosition(xMot+100,yMot-15).setCaptionLabel("RIGHT").show();
    
    
    for (i=0;i<3;i++) {ServoSliderMIN[i].hide(); ServoSliderMAX[i].hide();ServoSliderC[i].hide();}
    for (i=0;i<8;i++) {RateSlider[i].hide(); checkboxRev[i].hide();}
    ServoSliderMIN[7].hide(); ServoSliderMAX[7].hide();ServoSliderC[7].hide();
    TxtRates.hide();
    BtServo[0].setLabel(" ").setSize(100,12).setColorBackground(black_);
    BtServo[1].setLabel(" ").setSize(100,12).setColorBackground(black_);
    BtServo[2].setLabel(" ").setSize(100,12).setColorBackground(black_);
    BtServo[7].setLabel(" ").setSize(100,12).setColorBackground(black_);
    BtServo[3].setLabel("  NICK").setSize(100,12);
    BtServo[4].setLabel("  LEFT").setSize(100,12);
    BtServo[5].setLabel("  YAW");
    BtServo[6].setLabel("  RIGHT").setSize(100,12);
    
  } else if (multiType == HELI_90_DEG) { //Heli 90 
  
    for (i=0;i<3;i++) {
      ServoSliderMIN[i].hide(); ServoSliderMAX[i].hide();ServoSliderC[i].hide();
      BtServo[i].setLabel("").setSize(90,12).setColorBackground(black_);
      RateSlider[i].hide(); checkboxRev[i].hide();}

    RateSlider[5].hide();
    ServoSliderMIN[7].hide(); ServoSliderMAX[7].hide();ServoSliderC[7].hide();RateSlider[7].hide();
      
    BtServo[3].setLabel("  NICK").setSize(60,12);
    BtServo[4].setLabel("  ROLL").setSize(60,12);
    BtServo[5].setLabel("  YAW");
    BtServo[6].setLabel("  Coll").setSize(60,12);
    BtServo[7].setLabel("").setSize(90,12).setColorBackground(black_);
    TxtRates.hide();
    
    
    // HeliGraphics    
    float scalesize=size*0.8;
    // Rotor
    ellipse(0, 0, 2*scalesize, 2*scalesize);
    // Body  
    line(0,1.5*scalesize,  -2,-0.5*scalesize); line(0,1.5*scalesize,  2,-0.5*scalesize); line( -2,-0.5*scalesize,  2,-0.5*scalesize);    
    // Fin 
    float finpos = scalesize * 1.3;
    int HFin=5;
    int LFin=10;  
    line(0,finpos-3,0,  0,finpos+7,-LFin); line(0,finpos+7,-LFin,  0,finpos+10,-LFin);line(0,finpos+10,-LFin,  0,finpos+5,0); 
    line(0,finpos-3,0,  0,finpos,HFin); line(0,finpos,HFin,  0,finpos+5,HFin);line(0,finpos+5,HFin,  0,finpos+5,0); 
 
     // Stab
    line(-(scalesize*0.3),scalesize,  (scalesize*0.3),scalesize);   line(-(scalesize*0.3),scalesize+3, (scalesize*0.3),scalesize+3); 
    line(-(scalesize*0.3),scalesize, -(scalesize*0.3),scalesize+3); line((scalesize*0.3),scalesize,    (scalesize*0.3),scalesize+3);  
 
    noLights();
    textFont(font12);
    text("Heli 90", -16,-50);camera();popMatrix();
        
    // Sliders
    servoSliderV[7].setPosition(xMot,yMot-15)    .setCaptionLabel("Thro").show();     
    servoSliderV[4].setPosition(xMot+120,yMot-15).setCaptionLabel("ROLL").show(); 
    servoSliderV[3].setPosition(xMot+80,yMot+10) .setCaptionLabel("Nick").show();
    servoSliderH[5].setPosition(xMot+15,yMot+130).setCaptionLabel("Yaw") .show();
    servoSliderV[6].setPosition(xMot+40,yMot)    .setCaptionLabel("COLL").show();
  }  else if (multiType == VTAIL4) { //Vtail   
    drawMotor(+0.55*size, +size, byteMP[0], 'R');
    drawMotor(     +size, -size, byteMP[1], 'L');
    drawMotor(-0.55*size, +size, byteMP[2], 'L');
    drawMotor(     -size, -size, byteMP[3], 'R');
    line(-0.55*size,size,0,0);line(+0.55*size,size,0,0);    
    line(-size,-size, 0,0); line(+size,-size, 0,0);  
    noLights();
    textFont(font12);
    text("Vtail", -10,-50);camera();popMatrix();
    motSlider[0].setPosition(xMot+80,yMot+70 ).setHeight(60).setCaptionLabel("REAR_R").show();
    motSlider[1].setPosition(xMot+100,yMot-15).setHeight(60).setCaptionLabel("RIGHT" ).show();
    motSlider[2].setPosition(xMot+25,yMot+70 ).setHeight(60).setCaptionLabel("REAR_L").show();
    drawMotor(+size, +0.55*size, byteMP[0], 'L');
    motSlider[3].setPosition(xMot+2,yMot-15  ).setHeight(60).setCaptionLabel("LEFT"  ).show(); 
    
    motToggle[0].setPosition(xMot+70-MotToggleMove,yMot+75).show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot-15).show();
    motToggle[2].setPosition(xMot+40-MotToggleMove,yMot+75).show();
    motToggle[3].setPosition(xMot+10-MotToggleMove,yMot-15).show();
} else if(multiType == PPM_TO_SERVO) { //PPM to 8 servos    
    noLights();
    textFont(font12);
    text("PPM to 8 servos", -40, -50);
    camera();
    popMatrix();
    int ind=-5;
    for (i=0;i<8;i++) {servoSliderH[i].setPosition(xMot, yMot+ind).setCaptionLabel("CH "+(i+1)).show();
    BtServo[i].setLabel("   CH "+(i+1));
    ind+=20; }
  } else if (multiType == DUALCOPTER) { //Dualcopter
    float Span = size*1.3;  
    float VingRoot = Span*0.25;  
    // Stab
    line(0,VingRoot,  (size*0.4),size);   line(-(size*0.4),size+5,  (size*0.4),size+5); 
    line(0,VingRoot,  -(size*0.4),size);
    line(-(size*0.4),size,  -(size*0.4),size+5); line((size*0.4),size,  (size*0.4),size+5);     
    // Body  
    line(-2,size,  -2,-size+5); line(2,size,  2,-size+5); line( -2,-size+5,  2,-size+5);    
    // Fins
    line(0,VingRoot-3,0,  0,size,15);  line(0,size,15,  0,size+5,15);line(0,size+5,15,  0,size+5,0);   
    line(0,VingRoot-3,0,  0,size,-15); line(0,size,-15,  0,size+5,-15);line(0,size+5,-15,  0,size+5,0);       
    noLights();
    textFont(font12);
    text("Dualcopter", -30,-50);camera();popMatrix();
  
//    servoSliderH[3].setPosition(xMot,yMot-5) .setCaptionLabel("N/A").show();
    servoSliderH[4].setPosition(xMot,yMot+55).setCaptionLabel("PITCH").show();
    servoSliderH[5].setPosition(xMot,yMot+25).setCaptionLabel("ROLL").show();
    servoSliderH[6].setPosition(xMot,yMot+85).setCaptionLabel("M 1").show();
    servoSliderH[7].setPosition(xMot,yMot+115).setCaptionLabel("M 0").show();
    
   if(toggleWing){
      int Step=yServ;
      ServoSliderC[5].show().setPosition(xServ+100 ,Step+0+60);//.setCaptionLabel("  Center")
      ServoSliderMIN[5].show().setCaptionLabel("  Min") .setPosition(xServ+100  ,Step+0+80);
      ServoSliderMAX[5].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+0+80);
      Step+=10;
      ServoSliderC[4].show().setPosition(xServ+100 ,Step+80+60);// .show().setCaptionLabel("  Center") 
      ServoSliderMIN[4].show().setCaptionLabel("  Min") .setPosition(xServ+100  ,Step+80+80);
      ServoSliderMAX[4].show().setCaptionLabel("  Max") .setPosition(xServ+180 ,Step+80+80);
    }
    
    motToggle[0].setPosition(xMot-MotToggleMove,yMot+30).setCaptionLabel("").show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot+30).setCaptionLabel("").show();
  } else if (multiType == SINGLECOPTER) {
    float Span = size*1.3;  
    float VingRoot = Span*0.25;  
    // Stab
    line(0,VingRoot,  (size*0.4),size);   line(-(size*0.4),size+5,  (size*0.4),size+5); 
    line(0,VingRoot,  -(size*0.4),size);
    line(-(size*0.4),size,  -(size*0.4),size+5); line((size*0.4),size,  (size*0.4),size+5);     
    // Body  
    line(-2,size,  -2,-size+5); line(2,size,  2,-size+5); line( -2,-size+5,  2,-size+5);    
    // Fins
    line(0,VingRoot-3,0,  0,size,15);  line(0,size,15,  0,size+5,15);line(0,size+5,15,  0,size+5,0);   
    line(0,VingRoot-3,0,  0,size,-15); line(0,size,-15,  0,size+5,-15);line(0,size+5,-15,  0,size+5,0);       
    noLights();
    textFont(font12);
    text("SINGLECOPTER", -30,-50);camera();popMatrix();
  
    servoSliderH[3].setPosition(xMot,yMot-5) .setCaptionLabel("Right").show();
    servoSliderH[4].setPosition(xMot,yMot+25).setCaptionLabel("Left").show();
    servoSliderH[5].setPosition(xMot,yMot+55).setCaptionLabel("Front").show();    
    servoSliderH[6].setPosition(xMot,yMot+85).setCaptionLabel("Rear").show();
    servoSliderH[7].setPosition(xMot,yMot+115).setCaptionLabel("Motor").show();
    if(toggleWing){
      for(i=3;i<7;i++){ServoSliderC[i].show();ServoSliderMIN[i].show();ServoSliderMAX[i].show();}
    }
    
    motToggle[0].setPosition(xMot-MotToggleMove,yMot+30).setCaptionLabel("").show();
    motToggle[1].setPosition(xMot+100-MotToggleMove,yMot+30).setCaptionLabel("").show();
  } else {
    noLights();camera();popMatrix();
  }


  
  // ---------------------------------------------------------------------------------------------
  // Fly Level Control Instruments
  // ---------------------------------------------------------------------------------------------
  // info angles
  fill(255,255,127);
  textFont(font12);
  text((int)angy + "°", xLevelObj+38, yLevelObj+78); //pitch
  text((int)angx + "°", xLevelObj-62, yLevelObj+78); //roll

  pushMatrix();
  translate(xLevelObj-34,yLevelObj+112);
  fill(50,50,50);
  noStroke();
  ellipse(0,0,66,66);
  rotate(a);
  fill(255,255,127);
  textFont(font12);text("ROLL", -13, 15);
  strokeWeight(1.5);
  stroke(127,127,127);
  line(-30,1,30,1);
  stroke(255,255,255);
  line(-30,0,+30,0);line(0,0,0,-10);
  popMatrix();
  
  pushMatrix();
  translate(xLevelObj+34,yLevelObj+112);
  fill(50,50,50);
  noStroke();
  ellipse(0,0,66,66);
  rotate(b);
  fill(255,255,127);
  textFont(font12);text("PITCH", -18, 15);
  strokeWeight(1.5);
  stroke(127,127,127);
  line(-30,1,30,1);
  stroke(255,255,255);
  line(-30,0,30,0);line(30,0,30-size/6 ,size/6);line(+30,0,30-size/6 ,-size/6);  
  popMatrix();
 
  // ---------------------------------------------------------------------------------------------
  // Magnetron Combi Fly Level Control
  // ---------------------------------------------------------------------------------------------
  horizonInstrSize=68;
  angyLevelControl=((angy<-horizonInstrSize) ? -horizonInstrSize : (angy>horizonInstrSize) ? horizonInstrSize : angy);
  pushMatrix();
  translate(xLevelObj,yLevelObj);
  noStroke();
  // instrument background
  fill(50,50,50);
  ellipse(0,0,150,150);
  // full instrument
  rotate(-a);
  rectMode(CORNER);
  // outer border
  strokeWeight(1);
  stroke(90,90,90);
  //border ext
  arc(0,0,140,140,0,TWO_PI);
  stroke(190,190,190);
  //border int
  arc(0,0,138,138,0,TWO_PI);
  // inner quadrant
  strokeWeight(1);
  stroke(255,255,255);
  fill(124,73,31);
  //earth
  float angle = acos(angyLevelControl/horizonInstrSize);
  arc(0,0,136,136,0,TWO_PI);
  fill(38,139,224); 
  //sky 
  arc(0,0,136,136,HALF_PI-angle+PI,HALF_PI+angle+PI);
  float x = sin(angle)*horizonInstrSize;
  if (angy>0) 
    fill(124,73,31);
  noStroke();   
  triangle(0,0,x,-angyLevelControl,-x,-angyLevelControl);
  // inner lines
  strokeWeight(1);
  for(i=0;i<8;i++) {
    j=i*15;
    if (angy<=(35-j) && angy>=(-65-j)) {
      stroke(255,255,255); line(-30,-15-j-angy,30,-15-j-angy); // up line
      fill(255,255,255);
      textFont(font9);
      text("+" + (i+1) + "0", 34, -12-j-angy); //  up value
      text("+" + (i+1) + "0", -48, -12-j-angy); //  up value
    }
    if (angy<=(42-j) && angy>=(-58-j)) {
      stroke(167,167,167); line(-20,-7-j-angy,20,-7-j-angy); // up semi-line
    }
    if (angy<=(65+j) && angy>=(-35+j)) {
      stroke(255,255,255); line(-30,15+j-angy,30,15+j-angy); // down line
      fill(255,255,255);
      textFont(font9);
      text("-" + (i+1) + "0", 34, 17+j-angy); //  down value
      text("-" + (i+1) + "0", -48, 17+j-angy); //  down value
    }
    if (angy<=(58+j) && angy>=(-42+j)) {
      stroke(127,127,127); line(-20,7+j-angy,20,7+j-angy); // down semi-line
    }
  }
  strokeWeight(2);
  stroke(255,255,255);
  if (angy<=50 && angy>=-50) {
    line(-40,-angy,40,-angy); //center line
    fill(255,255,255);
    textFont(font9);
    text("0", 34, 4-angy); // center
    text("0", -39, 4-angy); // center
  }

  // lateral arrows
  strokeWeight(1);
  // down fixed triangle
  stroke(60,60,60);
  fill(180,180,180,255);

  triangle(-horizonInstrSize,-8,-horizonInstrSize,8,-55,0);
  triangle(horizonInstrSize,-8,horizonInstrSize,8,55,0);

  // center
  strokeWeight(1);
  stroke(255,0,0);
  line(-20,0,-5,0); line(-5,0,-5,5);
  line(5,0,20,0); line(5,0,5,5);
  line(0,-5,0,5);
  popMatrix();


  // ---------------------------------------------------------------------------------------------
  // Compass Section
  // ---------------------------------------------------------------------------------------------
  pushMatrix();
  translate(xCompass,yCompass);
  // Compass Background
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  size=29;
  rect(-size*2.5,-size*2.5,size*2.5,size*2.5);
  // GPS quadrant
  strokeWeight(1.5);
  if (GPS_update == 1) {
    fill(125);stroke(125);
  } else {
    fill(160);stroke(160);
  }
  ellipse(0,  0,   4*size+7, 4*size+7);
  // GPS rotating pointer
  rotate(GPS_directionToHome*PI/180);
  strokeWeight(4);stroke(255,255,100);line(0,0, 0,-2.4*size);line(0,-2.4*size, -5 ,-2.4*size+10); line(0,-2.4*size, +5 ,-2.4*size+10);  
  rotate(-GPS_directionToHome*PI/180);
  // compass quadrant
  strokeWeight(1.5);fill(0);stroke(0);
  ellipse(0,  0,   2.6*size+7, 2.6*size+7);
  // Compass rotating pointer
  stroke(255);
  rotate(head*PI/180);
  line(0,size*0.2, 0,-size*1.3); line(0,-size*1.3, -5 ,-size*1.3+10); line(0,-size*1.3, +5 ,-size*1.3+10);
  popMatrix();
  // angles 
  for (i=0;i<=12;i++) {
    angCalc=i*PI/6;
    if (i%3!=0) {
      stroke(75);
      line(xCompass+cos(angCalc)*size*2,yCompass+sin(angCalc)*size*2,xCompass+cos(angCalc)*size*1.6,yCompass+sin(angCalc)*size*1.6);
    } else {
      stroke(255);
      line(xCompass+cos(angCalc)*size*2.2,yCompass+sin(angCalc)*size*2.2,xCompass+cos(angCalc)*size*1.9,yCompass+sin(angCalc)*size*1.9);
    }
  }
  textFont(font15);
  text("N", xCompass-5, yCompass-22-size*0.9);text("S", xCompass-5, yCompass+32+size*0.9);
  text("W", xCompass-33-size*0.9, yCompass+6);text("E", xCompass+21+size*0.9, yCompass+6);
  // head indicator
  textFont(font12);
  noStroke();
  fill(80,80,80,130);
  rect(xCompass-22,yCompass-8,xCompass+22,yCompass+9);
  fill(255,255,127);
  text(head + "°",xCompass-11-(head>=10.0 ? (head>=100.0 ? 6 : 3) : 0),yCompass+6);
  // GPS direction indicator
  fill(255,255,0);
  text(GPS_directionToHome + "°",xCompass-6-size*2.1,yCompass+7+size*2);
  // GPS fix
  if (GPS_fix==0) {
     fill(127,0,0);
  } else {
     fill(0,255,0);
  }
  //ellipse(xCompass+3+size*2.1,yCompass+3+size*2,12,12);
  rect(xCompass-28+size*2.1,yCompass+1+size*2,xCompass+9+size*2.1,yCompass+13+size*2);
  textFont(font9);
  if (GPS_fix==0) {
    fill(255,255,0);
  } else {
    fill(0,50,0);
  }
  text("GPS_fix",xCompass-27+size*2.1,yCompass+10+size*2);


  // ---------------------------------------------------------------------------------------------
  // GRAPH
  // ---------------------------------------------------------------------------------------------
  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0); if (axGraph) g_graph.drawLine(accROLL, -1000, +1000);
  stroke(0, 255, 0); if (ayGraph) g_graph.drawLine(accPITCH, -1000, +1000);
  stroke(0, 0, 255);
  if (azGraph) {
    if (scaleSlider.value()<2){ g_graph.drawLine(accYAW, -1000, +1000);
    } else{ g_graph.drawLine(accYAW, 200*scaleSlider.value()-1000,200*scaleSlider.value()+500);}
  }
  
  float altMin = (altData.getMinVal() + altData.getRange() / 2) - 100;
  float altMax = (altData.getMaxVal() + altData.getRange() / 2) + 100;

  stroke(200, 200, 0);  if (gxGraph)   g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);  if (gyGraph)   g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);  if (gzGraph)   g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);if (altGraph)  g_graph.drawLine(altData, altMin, altMax);
  stroke(225, 225, 125);if (headGraph) g_graph.drawLine(headData, -370, +370);
  stroke(50, 100, 150); if (magxGraph) g_graph.drawLine(magxData, -500, +500);
  stroke(100, 50, 150); if (magyGraph) g_graph.drawLine(magyData, -500, +500);
  stroke(150, 100, 50); if (magzGraph) g_graph.drawLine(magzData, -500, +500);

  stroke(200, 50, 0); if (debug1Graph)  g_graph.drawLine(debug1Data, -5000, +5000);
  stroke(0, 200, 50); if (debug2Graph)  g_graph.drawLine(debug2Data, -5000, +5000);
  stroke(50, 0, 200); if (debug3Graph)  g_graph.drawLine(debug3Data, -5000, +5000);
  stroke(0, 0, 0);    if (debug4Graph)  g_graph.drawLine(debug4Data, -5000, +5000);

  // ------------------------------------------------------------------------
  // Draw background control boxes
  // ------------------------------------------------------------------------
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  // motor background
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
  // rc background
  rect(xRC-5,yRC-5, xRC+145, yRC+120);
  // param background
  rect(xParam,yParam, xParam+555, yParam+280);
if(!hideDraw){
  int xSens1       = xParam + 80;
  int ySens1       = yParam + 220;
  stroke(255);
  a=min(confRC_RATE.value(),1);
  b=confRC_EXPO.value();
  strokeWeight(1);
  line(xSens1,ySens1,xSens1,ySens1+35);
  line(xSens1,ySens1+35,xSens1+70,ySens1+35);
  strokeWeight(3);stroke(30,120,30);
  
  int lookupR[] = new int[6];
  for(i=0;i<6;i++) lookupR[i] = int( (2500+b*100*(i*i-25))*i*a*100/2500 );
  
  for(i=0;i<70;i++) {
    int tmp = 500/70*i;
    int tmp2 = tmp/100;
    int rccommand = 2*lookupR[tmp2] + 2*(tmp-tmp2*100) * (lookupR[tmp2+1]-lookupR[tmp2]) / 100;
    val = rccommand*70/1000;
    point(xSens1+i,ySens1+(70-val)*3.5/7);
  }
  if (confRC_RATE.value()>1) { 
    stroke(220,100,100);
    ellipse(xSens1+70, ySens1, 7, 7);
  }
  
  a=throttle_MID.value();
  b=throttle_EXPO.value();
  
  int xSens2       = xParam + 80;
  int ySens2       = yParam + 180;
  strokeWeight(1);stroke(255);
  line(xSens2,ySens2,xSens2,ySens2+35);
  line(xSens2,ySens2+35,xSens2+70,ySens2+35);
  strokeWeight(3);stroke(30,100,250);

  int lookupT[] = new int[11];
  for(i=0;i<11;i++) {
    int mid = int(100*a);
    int expo = int(100*b);
    int tmp = 10*i-mid;
    int y=1;
    if (tmp>0) y = 100-mid;
    if (tmp<0) y = mid;
    lookupT[i] = int( 10*mid +  tmp*( 100-expo+tmp*tmp*expo/(y*y) )/10  );
  }
  for(i=0;i<70;i++) {
    int tmp = 1000/70*i;
    int tmp2 = tmp/100;
    int rccommand = lookupT[tmp2] + (tmp-tmp2*100) * (lookupT[tmp2+1]-lookupT[tmp2]) / 100;
    val = rccommand*70/1000;
    point(xSens2+i,ySens2+(70-val)*3.5/7);
  }
  line(xSens2+(max(1100,RCChan[RCThro])-1100)*70/900,ySens2+25,xSens2+(max(1100,RCChan[RCThro])-1100)*70/900,ySens2+35);

  fill(255);
  textFont(font15);    
  text("P",xParam+45,yParam+15);text("I",xParam+90,yParam+15);text("D",xParam+130,yParam+15);
  textFont(font8);
  text("PITCH",xSens1+2,ySens1+10);
  text("ROLL",xSens1+2,ySens1+20);
  text("THROT",xSens2+2,ySens2+10);
  textFont(font12);
  text("RATE",xParam+3,yParam+232);
  text("EXPO",xParam+3,yParam+249);
  text("MID",xParam+3,yParam+193);
  text("EXPO",xParam+3,yParam+210);
  text("RATE",xParam+160,yParam+15);
  text("ROLL",xParam+3,yParam+32);
  text("PITCH",xParam+3,yParam+32+1*17);
  text("YAW",xParam+3,yParam+32+2*17);
  text("ALT",xParam+3,yParam+32+3*17);
  text("Pos",xParam+3,yParam+32+4*17);
  text("PosR",xParam+3,yParam+32+5*17);
  text("NavR",xParam+3,yParam+32+6*17);
  text("LEVEL",xParam+1,yParam+32+7*17);
  text("MAG",xParam+3,yParam+32+8*17);
  text("T P A",xParam+215,yParam+15);
  
  text("AUX1",xBox+55,yBox+5);text("AUX2",xBox+105,yBox+5);text("AUX3",xBox+165,yBox+5);text("AUX4",xBox+218,yBox+5);
  textFont(font8);
  text("LOW",xBox+37,yBox+15);text("MID",xBox+57,yBox+15);text("HIGH",xBox+74,yBox+15);
  text("L",xBox+100,yBox+15);text("M",xBox+118,yBox+15);text("H",xBox+136,yBox+15);
  text("L",xBox+154,yBox+15);text("M",xBox+174,yBox+15);text("H",xBox+194,yBox+15);
  text("L",xBox+212,yBox+15);text("M",xBox+232,yBox+15);text("H",xBox+252,yBox+15);
}
  pushMatrix();
  translate(0,0,0);
  popMatrix();
  if (versionMisMatch == 1) {textFont(font15);fill(#000000);text("GUI vs. Arduino: Version or Buffer size mismatch",180,420); return;}
}

void drawMotor(float x1, float y1, int mot_num, char dir) {   //Code by Danal
  float size = 30.0;
  pushStyle();
  float d = 0;
  if (dir == 'L') {d = +5; fill(254, 221, 44);} 
  if (dir == 'R') {d = -5; fill(256, 152, 12);}
  ellipse(x1, y1, size, size);
  textFont(font15);
  textAlign(CENTER);
  fill(0,0,0);
  text(mot_num,x1,y1+5,3);
  float y2 = y1-(size/2);
  stroke(255,0,0);
  line(x1, y2, 3, x1+d, y2-5, 3);
  line(x1, y2, 3, x1+d, y2+5, 3);
  line(x1, y2, 3, x1+d*2, y2, 3); 
  popStyle();
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {altGraph = theFlag;}
void HEAD(boolean theFlag) {headGraph = theFlag;}
void MAGX(boolean theFlag) {magxGraph = theFlag;}
void MAGY(boolean theFlag) {magyGraph = theFlag;}
void MAGZ(boolean theFlag) {magzGraph = theFlag;}
void DEBUG1(boolean theFlag) {debug1Graph = theFlag;}
void DEBUG2(boolean theFlag) {debug2Graph = theFlag;}
void DEBUG3(boolean theFlag) {debug3Graph = theFlag;}
void DEBUG4(boolean theFlag) {debug4Graph = theFlag;}

String ActiveTab="default";
public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
  if (theEvent.isGroup()) if (theEvent.name()=="baudList") GUI_BaudRate=(int)(theEvent.group().value());  // Set GUI_BaudRate to selected.
  if (theEvent.isTab()) { ActiveTab= theEvent.getTab().getName();  println("Switched to: "+ActiveTab);
    int tabN= +theEvent.getTab().getId();
    scaleSlider.moveTo(ActiveTab);
  
    if(tabN != 4) {// Don't show in Tab 4
      btnQConnect.moveTo(ActiveTab);
      buttonWRITE.moveTo(ActiveTab).hide();
      buttonREAD.moveTo(ActiveTab);
      buttonRESET.moveTo(ActiveTab);
    }
    for( i=0;i<8;i++) {
      TX_StickSlider[i].moveTo(ActiveTab);
      motSlider[i]   .moveTo(ActiveTab);
      servoSliderH[i].moveTo(ActiveTab);
      servoSliderV[i].moveTo(ActiveTab);
    }
    motorControlSlider.moveTo(ActiveTab);

    if(!Mag_)confINF[6].hide(); 
    if( tabN !=3 ) { txtlblWhichcom.moveTo(ActiveTab);commListbox.moveTo(ActiveTab);}
    if( tabN ==2 && gimbal && !gimbalConfig) toggleRead=true;
    if( tabN !=2 ){ toggleLive=false; buttonWRITE.show();}
    if( tabN ==1 ){hideDraw=false;}else{hideDraw=true;} // Hide grapics in all other tabs
    if( tabN ==4 ){
      motorControlSlider.show().setValue(1000);
      toggleMotor = true;
    } else {
      motorControlSlider.hide();
      toggleMotor = false;
    }
  }
}

public void bSTART() {
  if(graphEnabled == false) {return;}
  graph_on=1;
  toggleRead=true;
  g_serial.clear();
}

public void bSTOP() {
  graph_on=0;
}

public void SETTING() {
  toggleSetSetting = true;
}
void GIMBAL(){
  toggleGimbal = !toggleGimbal;
  if(toggleGimbal){toggleServo=false;toggleWing=false;toggleTrigger=false;}
}
void TRIGGER(){
  toggleTrigger=!toggleTrigger;
  if(toggleTrigger){toggleServo=false;toggleWing=false;toggleGimbal=false;}
}

public void READ() {
//  toggleLive = false ; 
  toggleRead = true;
  toggleVbat=true;
}

public void RESET() {
  toggleReset = true;
}

public void WRITE() {
  toggleWrite = true;
}

public void LIVE_SERVO() {
  toggleLive = !toggleLive;
}

public void SAVE_WING() {
  SAVE_Servo();
}
public void SAVE_Servo() {
  sendRequestMSP(requestMSP(MSP_EEPROM_WRITE));
}

public void WING(){ 
  toggleWing = !toggleWing;
  toggleGimbal = false;
  toggleTrigger=false;
  toggleRead=true;
}

public void SERVO() {
  toggleServo = !toggleServo;
  toggleGimbal = false;
  toggleTrigger=false;
  toggleRead=true;
}

public void CALIB_ACC() {toggleCalibAcc = true;}
public void CALIB_MAG() {toggleCalibMag = true;}

// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  if (portValue < commListMax) {
    String portPos = Serial.list()[int(portValue)];
    txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
    g_serial = new Serial(this, portPos, GUI_BaudRate);
    SaveSerialPort(portPos);
    init_com=1;
    buttonSTART.setColorBackground(green_);buttonSTOP.setColorBackground(green_);buttonREAD.setColorBackground(green_);
    buttonRESET.setColorBackground(green_);commListbox.setColorBackground(green_);
    buttonCALIBRATE_ACC.setColorBackground(green_); buttonCALIBRATE_MAG.setColorBackground(green_);
    graphEnabled = true;
    g_serial.buffer(256);
    btnQConnect.hide();
  } else {
    txtlblWhichcom.setValue("Comm Closed");
    init_com=0;
    buttonSTART.setColorBackground(red_);buttonSTOP.setColorBackground(red_);commListbox.setColorBackground(red_);buttonSETTING.setColorBackground(red_);
    graphEnabled = false;
    init_com=0;
    g_serial.stop();
    btnQConnect.show();
  }
}

void SaveSerialPort(String port ) {
    output = createWriter(portnameFile);
    output.print( port + ';' + GUI_BaudRate); // Write the comport to the file
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
 }
 void  Eport_Servo(){
   READ();
   ExportServo=true;
 }

void SAVE_SERVO_CONFIG() {  // Save a config file for servos
    ExportServo=false;
    output = createWriter("Servos.txt");
    String sServo[];
    output.println( "/* Defaut Servo settings exported from MultiWiiConf.");
    output.println( "   Place the defines in config.h");
    output.println( "   The Values will default if Eeprom is reset from Gui. */\n");
    output.print( "#define  SERVO_MIN  {"); for( i=0;i<7;i++) { output.print( ServoMIN[i]);output.print(", "); }output.print( ServoMIN[7]);output.println("}");
    output.print( "#define  SERVO_MAX  {"); for( i=0;i<7;i++) { output.print( ServoMAX[i]);output.print(", "); }output.print( ServoMAX[7]);output.println("}");
    output.print( "#define  SERVO_MID  {"); for( i=0;i<7;i++) { output.print( ServoMID[i]);output.print(", "); }output.print( ServoMID[7]);output.println("}");
    output.print( "#define  FORCE_SERVO_RATES  {"); for( i=0;i<7;i++) { output.print( servoRATE[i]);output.print(", "); }output.print( servoRATE[7]);output.println("}");
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
    buttonExport.hide();
}

public void bQCONN(){
  ReadSerialPort();
  InitSerial(SerialPort);
  bSTART();
  toggleRead=true;
}

void Cau0(){ServoSliderC[2].setMin(4).setMax(10).setValue(4);CauClear(); BtAUX[0].setColorBackground(orange_);}
void Cau1(){ServoSliderC[2].setMin(4).setMax(10).setValue(5);CauClear(); BtAUX[1].setColorBackground(orange_);}
void Cau2(){ServoSliderC[2].setMin(4).setMax(10).setValue(6);CauClear(); BtAUX[2].setColorBackground(orange_);}
void Cau3(){ServoSliderC[2].setMin(4).setMax(10).setValue(7);CauClear(); BtAUX[3].setColorBackground(orange_);}
void Cau4(){ServoSliderC[2].setMin(Centerlimits[0]).setMax(Centerlimits[1]).setValue(1500);CauClear();}
void CauClear(){ for (i=0;i<4;i++) BtAUX[i].setColorBackground(red_);}

void ReadSerialPort() {
    reader = createReader(portnameFile);  
    String line; 
  try {
    line = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    line = null; }
  if (line == null) {
    // Stop reading because of an error or file is empty
    // Roll on with no input
    btnQConnect.hide();
   return;
  } else {
    String[] pieces = split(line, ';');
    int Port = int(pieces[0]);
    GUI_BaudRate= int(pieces[1]);
    if (commListMax==1){}
    String pPort=pieces[0];
    for( i=0;i<commListMax;i++) {
      String[] pn =  match(shortifyPortName(Serial.list()[i], 13),  pieces[0]);    
      if ( pn !=null){ SerialPort=i;}
    }
  }
}

//save the content of the model to a file
public void bSAVE() {
  updateModel();
  SwingUtilities.invokeLater(new Runnable(){
    public void run() {
     final JFileChooser fc = new JFileChooser() {

        private static final long serialVersionUID = 7919427933588163126L;

        public void approveSelection() {
            File f = getSelectedFile();
            if (f.exists() && getDialogType() == SAVE_DIALOG) {
                int result = JOptionPane.showConfirmDialog(this,
                        "The file exists, overwrite?", "Existing file",
                        JOptionPane.YES_NO_CANCEL_OPTION);
                switch (result) {
                case JOptionPane.YES_OPTION:
                    super.approveSelection();
                    return;
                case JOptionPane.CANCEL_OPTION:
                    cancelSelection();
                    return;
                default:
                    return;
                }
            }
            super.approveSelection();
        }
    };

      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new MwiFileFilter());
      int returnVal = fc.showSaveDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File file = fc.getSelectedFile();
        
        FileOutputStream out =null;
        String error = null;
        try{
          out = new FileOutputStream(file) ;
          MWI.conf.storeToXML(out, new Date().toString()); 
          JOptionPane.showMessageDialog(null,new StringBuffer().append("configuration saved : ").append(file.toURI()) );
        }catch(FileNotFoundException e){
         
          error = e.getCause().toString();
        }catch( IOException ioe){
                /*failed to write the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
                
          if (out!=null){
            try{
              out.close();
            }catch( IOException ioe){/*failed to close the file*/error = ioe.getCause().toString();}
          }
          if (error !=null){
            JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
      }
    }
  }
  );
}

// import the content of a file into the model
public void bIMPORT(){
  SwingUtilities.invokeLater(new Runnable(){
    public void run(){
      final JFileChooser fc = new JFileChooser();
      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new MwiFileFilter());
      int returnVal = fc.showOpenDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File file = fc.getSelectedFile();
        FileInputStream in = null;
        boolean completed = false;
        String error = null;
        try{
          in = new FileInputStream(file) ;
          MWI.conf.loadFromXML(in); 
          JOptionPane.showMessageDialog(null,new StringBuffer().append("configuration loaded : ").append(file.toURI()) );
          completed  = true;
          
        }catch(FileNotFoundException e){
                error = e.getCause().toString();

        }catch( IOException ioe){/*failed to read the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
          if (!completed){
                 // MWI.conf.clear();
                 // or we can set the properties with view values, sort of 'nothing happens'
                 updateModel();
          }
          updateView();
          if (in!=null){
            try{
              in.close();
            }catch( IOException ioe){/*failed to close the file*/}
          }
          
          if (error !=null){
                  JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
      }
    }
  }
  );
}

//update the model with the value view 
public void updateModel(){
        updateModelMSP_SET_RC_TUNING();
        updateModelMSP_SET_PID();
        updateModelMSP_SET_MISC();
}

public void updateModelMSP_SET_RC_TUNING(){
        MWI.setProperty("rc.rate",String.valueOf(confRC_RATE.value()));
        MWI.setProperty("rc.expo",String.valueOf(confRC_EXPO.value()));
        MWI.setProperty("rc.rollpitch.rate",String.valueOf(rollPitchRate.value()));
        MWI.setProperty("rc.yaw.rate",String.valueOf(yawRate.value()));
        MWI.setProperty("rc.throttle.rate",String.valueOf(dynamic_THR_PID.value()));
        MWI.setProperty("rc.throttle.mid",String.valueOf(throttle_MID.value()));
        MWI.setProperty("rc.throttle.expo",String.valueOf(throttle_EXPO.value()));
}

public void updateModelMSP_SET_PID(){
for( i=0;i<PIDITEMS;i++) {
    MWI.setProperty("pid."+i+".p",String.valueOf(confP[i].value()));
    MWI.setProperty("pid."+i+".i",String.valueOf(confI[i].value()));
    MWI.setProperty("pid."+i+".d",String.valueOf(confD[i].value()));
  }
}

public void updateModelMSP_SET_MISC(){
   MWI.setProperty("power.trigger",String.valueOf(round(confPowerTrigger.value())));
}


// use the model to update the value of the view
public void updateView(){
  // MSP_SET_RC_TUNING
  confRC_RATE.setValue(Float.valueOf(MWI.conf.getProperty("rc.rate")));
  confRC_EXPO.setValue(Float.valueOf(MWI.conf.getProperty("rc.expo")));
  rollPitchRate.setValue(Float.valueOf(MWI.conf.getProperty("rc.rollpitch.rate")));
  yawRate.setValue(Float.valueOf(MWI.conf.getProperty("rc.yaw.rate")));
  dynamic_THR_PID.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.rate")));
  throttle_MID.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.mid")));
  throttle_EXPO.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.expo")));
  
  // MSP_SET_PID
  for( i=0;i<PIDITEMS;i++) {
     confP[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".p"))) ;
     confI[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".i"))) ;
     confD[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".d"))) ;
  }

  // MSP_SET_MISC
  confPowerTrigger.setValue(Float.valueOf(MWI.conf.getProperty("power.trigger")));
}
      

//  our model
static class MWI{
private static Properties conf = new Properties();

  public static void setProperty(String key ,String value ){
    conf.setProperty( key,value );
  }

  public static String getProperty(String key ){
    return conf.getProperty( key,"0");
  }

  public static void clear(){
    conf= null; // help gc
    conf = new Properties();
  }
}

//********************************************************
//********************************************************
//********************************************************

class cDataArray {
  float[] m_data;
  int m_maxSize, m_startIndex = 0, m_endIndex = 0, m_curSize;
  
  cDataArray(int maxSize){
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  void addVal(float val) {
    m_data[m_endIndex] = val;
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize) {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    } else {
      m_curSize++;
    }
  }
  float getVal(int index) {return m_data[(m_startIndex+index)%m_maxSize];}
  int getCurSize(){return m_curSize;}
  int getMaxSize() {return m_maxSize;}
  float getMaxVal() {
    float res = 0.0;
    for( i=0; i<m_curSize-1; i++) if ((m_data[i] > res) || (i==0)) res = m_data[i];
    return res;
  }
  float getMinVal() {
    float res = 0.0;
    for( i=0; i<m_curSize-1; i++) if ((m_data[i] < res) || (i==0)) res = m_data[i];
    return res;
  }
  float getRange() {return getMaxVal() - getMinVal();}
}

// This class takes the data and helps graph it
class cGraph {
  float m_gWidth, m_gHeight, m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h) {
    m_gWidth     = w; m_gHeight    = h;
    m_gLeft      = x; m_gBottom    = y;
    m_gRight     = x + w;
    m_gTop       = y + h;
  }
  
  void drawGraphBox() {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange) {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for( i=0; i<data.getCurSize()-1; ++i) {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gTop-(((data.getVal(i)-(maxRange+minRange)/2)*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gTop-(((data.getVal(i+1)-(maxRange+minRange)/2 )*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}

public class MwiFileFilter extends FileFilter {
 public boolean accept(File f) {
   if(f != null) {
     if(f.isDirectory()) {
       return true;
     }
     String extension = getExtension(f);
     if("mwi".equals(extension)) {
       return true;
     };
   }
   return false;
 }
 public String getExtension(File f) {
   if(f != null) {
      String filename = f.getName();
      int i = filename.lastIndexOf('.');
      if(i>0 && i<filename.length()-1) {
        return filename.substring(i+1).toLowerCase();
      };
    }
    return null;
  } 
  public String getDescription() {return "*.mwi Multiwii configuration file";}   
}

public void bRXbind() { //Bind a Spektrum Satellite
  toggleRXbind = true;
}

// Test with Tabs....
public void addTabs(){
  controlP5.getTab("default")
  .activateEvent(true)
  .setLabel("MultiWii.com")
  .setId(1)
  ;
  controlP5.addTab("ServoSettings")
  .activateEvent(true)
  .setColorBackground(color(blue_))
  .setColorLabel(color(255))
  //.setColorActive(color(255,128,0))
  .setColorActive(color(green_))
  .setLabel("Servo")
  .setId(2)
  .hide()
  ;
  controlP5.addTab("Config")
  .activateEvent(true)
  .setColorBackground(color(blue_))
  .setColorLabel(color(255))
  //.setColorActive(color(255,128,0))
  .setColorActive(color(green_))
  .setLabel("Settings")
  .setId(3)
  //.hide()
  ;
  controlP5.addTab("Motors")
  .activateEvent(true)
  .setColorBackground(color(blue_))
  .setColorLabel(color(255))
  //.setColorActive(color(255,128,0))
  .setColorActive(color(green_))
  .setLabel("Motors")
  .setId(4)
  .hide()
  ;
  
  int tHeight = 20;
  if( pVersion == 1.5 ) {
    controlP5.window().setPositionOfTabs(6, 13);
    tHeight = 17;
  }
  
  controlP5.tab("default").setHeight(tHeight);
  controlP5.tab("ServoSettings").setHeight(tHeight);
  controlP5.tab("Config").setHeight(tHeight);
  controlP5.tab("Motors").setHeight(tHeight);
 }
 
// WebLinks
public void MagDecl() { link("http://magnetic-declination.com/");}
public void MWiiHome(){ link("http://multiwii.com/forum/");}
public void MWiiGet() { link("https://code.google.com/p/multiwii/downloads/list");}

 // Test with tooltips....
public void Tooltips(){
  controlP5.getTooltip().setDelay(300);
  controlP5.getTooltip().register("bQCONN","ComPort must be Selected First Time.") ;  
  controlP5.getTooltip().register("SETTING","Save Multiple settings.") ;
  controlP5.getTooltip().register("MOTORS","Send Motors values to Copter.") ;  
  }
