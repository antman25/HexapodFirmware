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


float body_init[3][6] =  {  {43.0F, 63.0F, 43.0F, -43.0F, -63.0F, -43.0F}, 
                            {82.0F,  0.0F,-82.0F,  82.0F,   0.0F, -82.0F}, 
                            { 0.0F,  0.0F,  0.0F,   0.0F,   0.0F,   0.0F}};
                            
float RF_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float RM_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float RR_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };

float LF_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float LM_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };
float LR_transform[3][3] ={ {0,0,0}, {0,0,0}, {0,0,0} };

GaitData Gait[6];
LegAngles Angles[6];
BodyRotate bRot;
BodyTranslate bTrans;

int GaitSeq = 0;

#define USBSerial Serial
#define HWSerial Serial1

const float CoxaLength = 29.0F;
const float FemurLength = 76.0F;
const float TibiaLength = 106.0F;

