

struct LegAngles
{
  float CoxaAngle;
  float FemurAngle;
  float TibiaAngle;
};

struct GaitData
{
  float x;
  float y;
  float z;
  float rot_z;
};

struct BodyTranslate
{
  float x;
  float y;
  float z;
};

struct BodyRotate
{
  float x;
  float y;
  float z;
};

enum LEG
{
    RF=0, 
    RM=1, 
    RR=2, 
    LF=3, 
    LM=4, 
    LR=5
};

byte PROGMEM CoxaPin[] = { 8, 4, 0, 24, 20, 16 };
byte PROGMEM FemurPin[] = { 9, 5, 1, 25, 21, 17 };
byte PROGMEM TibiaPin[] = { 10, 6, 2, 26, 22, 18 };

const float CoxaLength = 29.0F;
const float FemurLength = 76.0F;
const float TibiaLength = 106.0F;

const float FootInitX = 105.0F;
const float FootInitXYCos60 = 52.5F;
const float FootInitXYSin60 = 90.9327F;

const float BodyInitX = 63.0F;
const float BodyInitXYCos60 = 43.0F;
const float BodyInitXYSin60 = 82.0F;

float body_init[3][6] =  {  {43.0F, 63.0F, 43.0F, -43.0F, -63.0F, -43.0F}, 
                            {82.0F,  0.0F,-82.0F,  82.0F,   0.0F, -82.0F}, 
                            { 0.0F,  0.0F,  0.0F,   0.0F,   0.0F,   0.0F}};
                            
float foot_init[3][6] =  {  { FootInitXYCos60,  FootInitX,    FootInitXYCos60,   -FootInitXYCos60,  -FootInitX, -FootInitXYCos60}, 
                            { FootInitXYSin60,  0.0F,         -FootInitXYSin60,  FootInitXYSin60,  0,           -FootInitXYSin60}, 
                            { TibiaLength,      TibiaLength,  TibiaLength,       TibiaLength,      TibiaLength,  TibiaLength}};                            
                            
                            
                            
                            
/*const float RF_body_init[3][1] = { {BodyInitXYCos60},   {BodyInitXYSin60}, {0} };                          
const float RM_body_init[3][1] = { {BodyInitX},         {0},               {0} };
const float RR_body_init[3][1] = { {BodyInitXYCos60},   {-BodyInitXYSin60},{0} };
const float LF_body_init[3][1] = { {-BodyInitXYCos60},  {BodyInitXYSin60}, {0} };                          
const float LM_body_init[3][1] = { {-BodyInitX},        {0},               {0} };
const float LR_body_init[3][1] = { {-BodyInitXYCos60},  {-BodyInitXYSin60},{0} };  */                                                  
                            
/*const float RF_foot_init[3][1] = { {FootInitXYCos60},   {FootInitXYSin60}, {TibiaLength} };                          
const float RM_foot_init[3][1] = { {FootInitX},         {0},               {TibiaLength} };
const float RR_foot_init[3][1] = { {FootInitXYCos60},   {-FootInitXYSin60},{TibiaLength} };
const float LF_foot_init[3][1] = { {-FootInitXYCos60},  {FootInitXYSin60}, {TibiaLength} };                          
const float LM_foot_init[3][1] = { {-FootInitX},        {0},               {TibiaLength} };
const float LR_foot_init[3][1] = { {-FootInitXYCos60},  {-FootInitXYSin60},{TibiaLength} };*/
                            

float RF_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float RM_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float RR_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };

float LF_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float LM_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float LR_body_to_leg[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };


GaitData Gait[6];
LegAngles Angles[6];
BodyRotate bRot;
BodyTranslate bTrans;

int GaitSeq = 0;

float Xmove = 0.0F;
float Ymove = 0.0F;
float Zrot = 0.0F;
float LiftHeight = 0.0F;

#define USBSerial Serial
#define SSC32Serial Serial1
#define BlueToothSerial Serial2

#define RIPPLE                  0
#define RIPPLE_SMOOTH           1
#define AMBLE                   2
#define AMBLE_SMOOTH            3 
#define TRIPOD                  4

#define MOVING   ((Xspeed > 5 || Xspeed < -5) || (Yspeed > 5 || Yspeed < -5) || (Rspeed > 0.05 || Rspeed < -0.05))

#define STD_TRANSITION          100

#define CMD_BUFFER_SIZE         512

size_t cmd_buffer_ptr = 0;
uint8_t cmd_buffer[CMD_BUFFER_SIZE];
char *cmd_buffer_test;

union float2bytes { byte b[sizeof(float)]; float f; };

