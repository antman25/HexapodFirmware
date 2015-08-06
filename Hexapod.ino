#include "Arduino.h"
#include "common.h"

#include <string.h>
#include <SPI.h>
#include "utility/debug.h"
#include "utility/socket.h"
#include <Adafruit_CC3000.h>

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
Adafruit_CC3000_Server hexapodServer(LISTEN_PORT);

uint32_t startTime;

void MatrixPrint(float* A, int m, int n, char* label){
     // A = input matrix (m x n)
     int i,j;
     Serial.println(label);
     Serial.println(" Matrix values:");
     for (i=0; i<m; i++){
           for (j=0;j<n;j++){
                 Serial.print(A[n*i+j]);
                 Serial.print("; i=");
                 Serial.print(i);
                 Serial.print(" j=");
                 Serial.println(j);
           }
     }
}

void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
    {
      C[n*i+j]=0;
      for (k=0;k<p;k++)
        C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
    }
}

void MatrixAdd(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[n*i+j]=A[n*i+j]+B[n*i+j];
}

void MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A-B (m x n)
  int i, j;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[n*i+j]=A[n*i+j]-B[n*i+j];
}

void MatrixTranspose(float* A, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[m*j+i]=A[n*i+j];
}

void BuildRotationMatrixXYZ(float x_angle, float y_angle, float z_angle, float *result)
{
  // Convert all angles to radians
  float rad_x_angle = x_angle * DEG_TO_RAD;
  float rad_y_angle = y_angle * DEG_TO_RAD;
  float rad_z_angle = z_angle * DEG_TO_RAD;
  
  // Assume input angles are all 0 degrees
  
  float RxCos = 1.0F;
  float RxSin = 0.0F;
  
  float RyCos = 1.0F;
  float RySin = 0.0F;
  
  float RzCos = 1.0F;
  float RzSin = 0.0F;
  
  // Only calculate for non zero angles to save cpu cycles
  
  if (x_angle != 0.0F)
  {
    RxCos = cosf(rad_x_angle);
    RxSin = sinf(rad_x_angle);
  }
  
  if (y_angle != 0.0F)
  {
    RyCos = cosf(rad_y_angle);
    RySin = sinf(rad_y_angle);
  }
  
  if (z_angle != 0.0F)
  {
    RzCos = cosf(rad_z_angle);
    RzSin = sinf(rad_z_angle);
  }
  
  // Build rotation matrices 
  float Rx[3][3] = { {1,0,0}, { 0, RxCos, -RxSin }, {0, RxSin, RxCos} };
  float Ry[3][3] = { {RyCos,0,RySin}, {0,1,0}, {-RySin,0,RyCos}};
  float Rz[3][3] = { {RzCos, -RzSin, 0}, {RzSin, RzCos,0}, {0,0,1}};
  // Rxy holds the intermediate result for final Rxyz solution
  float Rxy[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
  // Calculate Rxy
  MatrixMultiply((float*)Rx,(float*)Ry,3,3,3,(float*)Rxy);
  // Calculate Rxyz
  MatrixMultiply((float*)Rxy,(float*)Rz,3,3,3,(float*)result);
}

void RotateXYZ(float *points, float x_angle, float y_angle, float z_angle, float *result)
{                
  float Rxyz[3][3] = {{0,0,0},
                      {0,0,0},
                      {0,0,0}};

  // Build rotation matrix based off supplied angles                         
  BuildRotationMatrixXYZ(x_angle,y_angle,z_angle,(float*)Rxyz);
  
  // Perform rotation on the 6-points of the body
  MatrixMultiply((float*)Rxyz,(float*)points,3,3,1,(float*)result);
}

void BuildTransforms()
{
  float temp[3][3] ={ {0,0,0}, 
                      {0,0,0}, 
                      {0,0,0}};
  // These rotation matrices are to convert from the leg coordinate system to the body coordinate system.                    
  BuildRotationMatrixXYZ(0,0,60,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RF_body_to_leg);
  
  BuildRotationMatrixXYZ(0,0,0,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RM_body_to_leg);
  
  BuildRotationMatrixXYZ(0,0,300,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RR_body_to_leg);
  
  BuildRotationMatrixXYZ(0,0,120,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LF_body_to_leg);
  
  BuildRotationMatrixXYZ(0,0,180, (float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LM_body_to_leg);
  
  BuildRotationMatrixXYZ(0,0,240,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LR_body_to_leg);
  

}

float* GetBodyToLegTransform (byte LegNr)
{
  switch (LegNr)
  {
    case RF:
      return (float*)RF_body_to_leg;
    case RM:
      return (float*)RM_body_to_leg;
    case RR:
      return (float*)RR_body_to_leg;
    case LF:
      return (float*)LF_body_to_leg;
    case LM:
      return (float*)LM_body_to_leg;
    case LR:
      return (float*)LR_body_to_leg;
  }
  return NULL;
}

short AngleToPWM(float angle)
{
    return (short)((((angle + 90) / 180.0) * 1800) + 600);
}

LegAngles LegIK(float FeetPosX, float FeetPosY, float FeetPosZ, byte LegNr)
{
  LegAngles result;
  float foot_vector[3][1] = { { FeetPosX }, { FeetPosY }, { FeetPosZ } };
  float foot_transform[3][1] = { {0}, {0}, {0} };
  
  MatrixMultiply(GetBodyToLegTransform(LegNr),(float*)foot_vector,3,3,1,(float*)foot_transform);
  
  /*USBSerial.print("X: ");
  USBSerial.print(foot_transform[0][0]);
  USBSerial.print("   Y: ");
  USBSerial.print(foot_transform[1][0]);
  USBSerial.print("   Z: ");
  USBSerial.println(foot_transform[2][0]);*/
  
  float foot_x = (CoxaLength + FemurLength) + foot_transform[0][0];
  float foot_y = foot_transform[1][0];  
  float foot_z = TibiaLength + foot_transform[2][0];
  
  
  /*USBSerial.print("Xt: ");
  USBSerial.print(foot_x);
  USBSerial.print("   Yt: ");
  USBSerial.print(foot_y);
  USBSerial.print("   Zt: ");
  USBSerial.println(foot_z);*/
  

  float L = sqrt( foot_z * foot_z + (foot_x - CoxaLength)*(foot_x - CoxaLength));
  float A1 = acosf(foot_z / L);
  float A2 = acosf( (L*L + FemurLength*FemurLength - TibiaLength*TibiaLength) / (2*L*FemurLength) );
  float A = (A1 + A2) * RAD_TO_DEG;
  float B = acosf( (FemurLength*FemurLength + TibiaLength*TibiaLength - L*L) / (2*FemurLength*TibiaLength) ) * RAD_TO_DEG;

  result.CoxaAngle = atan2f( foot_y , foot_x ) * RAD_TO_DEG;
  result.FemurAngle = -(A - 90.0F);
  result.TibiaAngle = 90.0F - B;
  
  /*USBSerial.print("CoxaAngle: ");
  USBSerial.print(result.CoxaAngle,DEC);
  USBSerial.print("   FemurAngle: ");
  USBSerial.print(result.FemurAngle,DEC);
  USBSerial.print("   TibiaAngle: ");
  USBSerial.println(result.TibiaAngle,DEC);*/
  
  return result;
}

void InitializeCommandBuffer()
{
  for (cmd_buffer_ptr = 0; cmd_buffer_ptr < CMD_BUFFER_SIZE;cmd_buffer_ptr++)
  {
    cmd_buffer[cmd_buffer_ptr] = 0;
  }
  cmd_buffer_ptr = 0;
}

void InitializePositions()
{
  for (int i = 0;i<6;i++)
  {
    Gait[i].x = 0;
    Gait[i].y = 0;
    Gait[i].z = 0;
    Gait[i].rot_z = 0;
  }
  
  bRot.x = 0.0F;
  bRot.y = 0.0F;
  bRot.z = 0.0F;
  bTrans.x = 0.0F;
  bTrans.y = 0.0F;
  bTrans.z = 0.0F;
}


void InitializeWifi()
{
  USBSerial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);

  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) 
  {
    Serial.println(F("Failed!"));
    while(1);
  }

  USBSerial.println(F("Connected!"));
  USBSerial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  while (! displayConnectionDetails()) 
  {
    delay(1000);
  }

  InitializeCommandBuffer();

  hexapodServer.begin();

  USBSerial.println(F("Listening for connections..."));
}





void UpdateLegs()
{
  unsigned long time = micros();

  LegAngles angles[6];
  float body_translate[3][1] = { {bTrans.x}, {bTrans.y},{bTrans.z} };
  
  for (int i=0;i<6;i++)
  {
    //Serial.print("Leg: ");
    //Serial.println(i,DEC);

    float body_rot_out[3][1] = { {0}, {0}, {0} };
    float body_diff[3][1] = { {0}, {0}, {0} };
    float body_diff_total[3][1] = { {0}, {0}, {0} };
    
    
    float body_init_pt[3][1] = { {body_init[0][i]}, {body_init[1][i]}, {body_init[2][i]} };
    //MatrixPrint((float*)body_init_pt,3,1,"\nbody_init_pt");   
    RotateXYZ((float*)body_init_pt, bRot.x, bRot.y, bRot.z,(float*)body_rot_out);
    //MatrixPrint((float*)body_rot_out,3,1,"\nbody_rot_out");
    
    MatrixSubtract((float*)body_init_pt, (float*)body_rot_out,3,1,(float*)body_diff);
    //MatrixPrint((float*)body_diff,3,1,"\nbody_diff");
    MatrixAdd((float*)body_diff, (float*)body_translate,3,1,(float*)body_diff_total);
    //MatrixPrint((float*)body_diff_total,3,1,"\nbody_diff_total");
    
    float foot_pt[3][1] = { {body_init[0][i] + foot_init[0][i]}, {body_init[1][i] + foot_init[1][i]}, {body_init[2][i] + foot_init[2][i]} };
    float foot_rot[3][1] = { {0}, {0}, {0} };
    float foot_diff[3][1] = { {0}, {0}, {0} };
    float foot_diff_total[3][1] = { {0}, {0}, {0} };
    float gait_trans[3][1] = { {Gait[i].x}, {Gait[i].y}, {Gait[i].z} };
    //MatrixPrint((float*)foot_pt,3,1,"\nfoot_pt");
    RotateXYZ((float*)foot_pt, 0, 0, Gait[i].rot_z,(float*)foot_rot);
    //MatrixPrint((float*)foot_rot,3,1,"\nfoot_rot");
    MatrixSubtract((float*)foot_rot, (float*)foot_pt,3,1,(float*)foot_diff);
    //MatrixPrint((float*)foot_diff,3,1,"\nfoot_diff");
    MatrixAdd((float*)foot_diff, (float*)gait_trans,3,1,(float*)foot_diff_total);
    //MatrixPrint((float*)foot_diff_total,3,1,"\nfoot_diff_total");
    
    float foot_ik[3][1] = { {0}, {0}, {0} };
    MatrixAdd((float*)body_diff_total, (float*)foot_diff_total,3,1,(float*)foot_ik);
    //MatrixPrint((float*)foot_ik,3,1,"\nfoot_ik");
    
    angles[i] = LegIK(foot_ik[0][0], foot_ik[0][1], foot_ik[0][2], i);
  }
  //unsigned long dtime = micros() - time;
  //USBSerial.print("Time: ");
  
  //USBSerial.println(dtime);
  
  setLegAngles(angles,120);
}

void setLegAngles(LegAngles *angles, word wMoveTime)
{
  word wCoxaSSCV = 1500;
  word wFemurSSCV = 1500;
  word wTibiaSSCV = 1500;
  
  for (int i = 0;i < 6;i++)
  {
    /*USBSerial.print("Leg: ");
    USBSerial.print(i);
    USBSerial.print("  ");
    USBSerial.print(angles[i].CoxaAngle);
    USBSerial.print("  ");
    USBSerial.print(angles[i].FemurAngle);
    USBSerial.print("  ");
    USBSerial.println(angles[i].TibiaAngle);*/
    
    
    if (i < 3)
    {
      wCoxaSSCV = AngleToPWM(angles[i].CoxaAngle);
      wFemurSSCV = AngleToPWM(angles[i].FemurAngle);
      wTibiaSSCV = AngleToPWM(-angles[i].TibiaAngle);
    }
    else
    {
      wCoxaSSCV = AngleToPWM(angles[i].CoxaAngle);
      wFemurSSCV = AngleToPWM(-angles[i].FemurAngle);
      wTibiaSSCV = AngleToPWM(angles[i].TibiaAngle);
    }
  
    HWSerial.print("#");
    HWSerial.print(pgm_read_byte(&CoxaPin[i]), DEC);
    HWSerial.print("P");
    HWSerial.print(wCoxaSSCV, DEC);
    HWSerial.print("#");
    HWSerial.print(pgm_read_byte(&FemurPin[i]), DEC);
    HWSerial.print("P");
    HWSerial.print(wFemurSSCV, DEC);
    HWSerial.print("#");
    HWSerial.print(pgm_read_byte(&TibiaPin[i]), DEC);
    HWSerial.print("P");
    HWSerial.print(wTibiaSSCV, DEC);
  }
  USBSerial.println("");
  HWSerial.print("T");
  HWSerial.println(wMoveTime, DEC);
  SSC32Wait(wMoveTime);
}

void SSC32Wait(word wMoveTime)
{
  uint32_t time = millis();
  while (millis() - time <= wMoveTime)
  {}
  
  //for (uint16_t count = 0;count < 16000;count++)
  /*while (1)
  {
    HWSerial.write("Q \r");
    delay(1);
    if (HWSerial.available() > 0) 
    {
      char incomingByte = HWSerial.read();
      //USBSerial.println(incomingByte);
      if (incomingByte == '.')
      {
        USBSerial.println("Done!");
        break;
      }       
    }
    delay(1);
  }*/
  
  uint32_t dtime = millis() - time;
  //USBSerial.print("Time: ");
  //USBSerial.println(dtime, DEC);
}

void UpdateGait()
{
  float XmoveDiv2 = Xmove / 2.0f;
  float XmoveDiv4 = Xmove / 4.0f;
  float YmoveDiv2 = Ymove / 2.0f;
  float YmoveDiv4 = Ymove / 4.0f;
  
  float ZrotDiv2 = Zrot / 2.0f;
  float ZrotDiv4 = Zrot / 4.0f;
  
  switch (GaitSeq)
  {
    case 0:
      Gait[RF].x = -XmoveDiv2;
      Gait[RF].y = -YmoveDiv2;
      Gait[RF].z = 0.0F;
      Gait[RF].rot_z = -ZrotDiv2;
      
      Gait[RM].x = XmoveDiv2;
      Gait[RM].y = YmoveDiv2;
      Gait[RM].z = 0.0F;
      Gait[RM].rot_z = ZrotDiv2;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 0.0F;
      Gait[RR].z = 0.0F;
      Gait[RR].rot_z = 0.0F;
      
      Gait[LF].x = XmoveDiv4;
      Gait[LF].y = YmoveDiv4;
      Gait[LF].z = 0.0F;
      Gait[LF].rot_z = ZrotDiv4;
      
      Gait[LM].x = -XmoveDiv4;
      Gait[LM].y = -YmoveDiv4;
      Gait[LM].z = 0.0F;
      Gait[LM].rot_z = -ZrotDiv4;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 0.0F;
      Gait[LR].z = -LiftHeight;
      Gait[LR].rot_z = 0.0F;

      break;
    case 1:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 0.0F;
      Gait[RF].z = -LiftHeight;
      Gait[RF].rot_z = 0.0F;
      
      Gait[RM].x = XmoveDiv4;
      Gait[RM].y = YmoveDiv4;
      Gait[RM].z = 0.0F;
      Gait[RM].rot_z = ZrotDiv4;
      
      Gait[RR].x = -XmoveDiv4;
      Gait[RR].y = -YmoveDiv4;
      Gait[RR].z = 0.0F;
      Gait[RR].rot_z = -ZrotDiv4;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 0.0F;
      Gait[LF].z = 0.0F;
      Gait[LF].rot_z = 0.0F;
      
      Gait[LM].x = -XmoveDiv2;
      Gait[LM].y = -YmoveDiv2;
      Gait[LM].z = 0.0F;
      Gait[LM].rot_z = -ZrotDiv2;
      
      Gait[LR].x = XmoveDiv2;
      Gait[LR].y = YmoveDiv2;
      Gait[LR].z = 0.0F;
      Gait[LR].rot_z = ZrotDiv2;
      break;
    case 2:
      Gait[RF].x = XmoveDiv2;
      Gait[RF].y = YmoveDiv2;
      Gait[RF].z = 0.0F;
      Gait[RF].rot_z = ZrotDiv2;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 0.0F;
      Gait[RM].z = 0.0F;
      Gait[RM].rot_z = 0.0F;
      
      Gait[RR].x = -XmoveDiv2;
      Gait[RR].y = -YmoveDiv2;
      Gait[RR].z = 0.0F;
      Gait[RR].rot_z = -ZrotDiv2;
      
      Gait[LF].x = -XmoveDiv4;
      Gait[LF].y = -YmoveDiv4;
      Gait[LF].z = 0.0F;
      Gait[LF].rot_z = -ZrotDiv4;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 0.0F;
      Gait[LM].z = -LiftHeight;
      Gait[LM].rot_z = 0.0F;
      
      Gait[LR].x = XmoveDiv4;
      Gait[LR].y = YmoveDiv4;
      Gait[LR].z = 0.0F;
      Gait[LR].rot_z = ZrotDiv4;
      break;
    case 3:
      Gait[RF].x = XmoveDiv4;
      Gait[RF].y = YmoveDiv4;
      Gait[RF].z = 0.0F;
      Gait[RF].rot_z = ZrotDiv4;
      
      Gait[RM].x = -XmoveDiv4;
      Gait[RM].y = -YmoveDiv4;
      Gait[RM].z = 0.0F;
      Gait[RM].rot_z = -ZrotDiv4;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 0.0F;
      Gait[RR].z = -30.0F;
      Gait[RR].rot_z = 0.0F;
      
      Gait[LF].x = -XmoveDiv2;
      Gait[LF].y = -YmoveDiv2;
      Gait[LF].z = 0.0F;
      Gait[LF].rot_z = -ZrotDiv2;
      
      Gait[LM].x = XmoveDiv2;
      Gait[LM].y = YmoveDiv2;
      Gait[LM].z = 0.0F;
      Gait[LM].rot_z = ZrotDiv2;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 0.0F;
      Gait[LR].z = 0.0F;
      Gait[LR].rot_z = 0.0F;
      break;
    case 4:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 0.0F;
      Gait[RF].z = 0.0F;
      Gait[RF].rot_z = 0.0F;
      
      Gait[RM].x = -XmoveDiv2;
      Gait[RM].y = -YmoveDiv2;
      Gait[RM].z = 0.0F;
      Gait[RM].rot_z = -ZrotDiv2;
      
      Gait[RR].x = XmoveDiv2;
      Gait[RR].y = YmoveDiv2;
      Gait[RR].z = 0.0F;
      Gait[RR].rot_z = ZrotDiv2;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 0.0F;
      Gait[LF].z = -LiftHeight;
      Gait[LF].rot_z = 0.0F;
      
      Gait[LM].x = XmoveDiv4;
      Gait[LM].y = YmoveDiv4;
      Gait[LM].z = 0.0F;
      Gait[LM].rot_z = ZrotDiv4;
      
      Gait[LR].x = -XmoveDiv4;
      Gait[LR].y = -YmoveDiv4;
      Gait[LR].z = 0.0F;
      Gait[LR].rot_z = -ZrotDiv4;
      break;
    case 5:
      Gait[RF].x = -XmoveDiv4;
      Gait[RF].y = -YmoveDiv4;
      Gait[RF].z = 0.0F;
      Gait[RF].rot_z = -ZrotDiv4;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 0.0F;
      Gait[RM].z = -LiftHeight;
      Gait[RM].rot_z = 0.0F;
      
      Gait[RR].x = XmoveDiv4;
      Gait[RR].y = YmoveDiv4;
      Gait[RR].z = 0.0F;
      Gait[RR].rot_z = ZrotDiv4;
      
      Gait[LF].x = XmoveDiv2;
      Gait[LF].y = YmoveDiv2;
      Gait[LF].z = 0.0F;
      Gait[LF].rot_z = ZrotDiv2;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 0.0F;
      Gait[LM].z = 0.0F;
      Gait[LM].rot_z = 0.0F;
      
      Gait[LR].x = -XmoveDiv2;
      Gait[LR].y = -YmoveDiv2;
      Gait[LR].z = 0.0F;
      Gait[LR].rot_z = -ZrotDiv2;
      GaitSeq=-1;
      break;
      
    default:
      GaitSeq=0;
      break;
  }
  GaitSeq++;
}

void PrintCharArray(uint8_t* input, uint16_t len)
{
  for (uint16_t i = 0;i < len;i++)
  {
    hexapodServer.write(input[i]);
  }
}

void GetWifiData()
{
  Adafruit_CC3000_ClientRef client = hexapodServer.available();
        
  if (client) 
  {
    // Check if there is data available to read.
    if (client.available() > 0) 
    {
      // Read a byte and write it to all clients.
      uint8_t ch = client.read();
      if (ch == '\n')
      {
        //hexapodServer.write(ch);
  	//Serial.println(F("Processing command"));
        ProcessCommand();
      }
      else
      {
        cmd_buffer[cmd_buffer_ptr++] = ch;
      }
      //String strCmdBuffer = String((char*)cmd_buffer);
      //USBSerial.println("Len: " + String(cmd_buffer_ptr));
    }
  }
}

void ProcessCommand()
{
	//hexapodServer.print("Processing: ");
	//PrintCharArray(cmd_buffer,cmd_buffer_ptr);
	//hexapodServer.println("");

	uint8_t cmdByte = cmd_buffer[0];
	uint16_t cmdLen = (cmd_buffer[1] << 8) | cmd_buffer[2];
	String strCmdBuffer = String((char*)cmd_buffer);
	
	switch (cmdByte)
	{
		case 'I':
			USBSerial.println(F("Setting all positions to 1500"));
			HWSerial.write("#0 P1500 #1 P1500 #2 P1500 #4 P1500 #5 P1500 #6 P1500 #8 P1500 #9 P1500 #10 P1500 #16 P1500 #17 P1500 #18 P1500 #20 P1500 #21 P1500 #22 P1500 #24 P1500 #25 P1500 #26 P1500 T1000\r");
			break;
		case 'R':
			USBSerial.println(F("Setting all offsets to 0"));
			HWSerial.write("#0 PO0 #1 PO0 #2 PO0 #4 PO0 #5 PO0 #6 PO0 #8 PO0 #9 PO0 #10 PO0 #16 PO0 #17 PO0 #18 PO0 #20 PO0 #21 PO0 #22 PO0 #24 PO0 #25 PO0 #26 PO0\r");
			break;
		case 'N':
			USBSerial.println(F("Initializing hexapod positions"));
			InitializePositions();
			break;
		case 'S':
			USBSerial.println(F("-----SSC Command-----"));
			USBSerial.println(strCmdBuffer);
			//for (int i = 1;i<cmd_buffer_ptr;i++)
			//	USBSerial.write(cmd_buffer[i]);

			//USBSerial.print("Command Len: ");
			//USBSerial.print(cmdLen);
			
			//USBSerial.write(&cmd_buffer+1,cmd_buffer_ptr-1);
			for (int i = 1;i<cmd_buffer_ptr;i++)
				HWSerial.write(cmd_buffer[i]);
			
			break;
		default:
			USBSerial.println(F("Unknown Command"));
			PrintCharArray(cmd_buffer,cmd_buffer_ptr);
	}

	InitializeCommandBuffer();
}


bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
 
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\n Netmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\n Gateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\n DHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\n DNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
 
void setup() {
  USBSerial.begin(115200);
  HWSerial.begin(115200);

  BuildTransforms();
  InitializePositions();
  //InitializeWifi();
  delay(3000);
  HWSerial.write("#0 PO0 #1 PO-25 #2 PO-50 #4 PO0 #5 PO25 #6 PO50 #8 PO0 #9 PO0 #10 PO50 #16 PO0 #17 PO50 #18 PO0 #20 PO0 #21 PO-50 #22 PO-50 #24 PO0 #25 PO50 #26 PO0\r");
  delay(5000);
  startTime = millis();
}

void loop() 
{
  uint32_t deltaTime = millis() - startTime;
  if (deltaTime < 10000)
  {
    Xmove = 0.0F;
    Ymove = 25.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 10000 && deltaTime < 20000)
  {
    Xmove = 25.0F;
    Ymove = 25.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 20000 && deltaTime < 30000)
  {
    Xmove = -25.0F;
    Ymove = -25.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 30000 && deltaTime < 40000)
  {
    Xmove = 25.0F;
    Ymove = 0.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 40000 && deltaTime < 50000)
  {
    Xmove = -25.0F;
    Ymove = 0.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 50000 && deltaTime < 75000)
  {
    Xmove = 0.0F;
    Ymove = 25.0F;
    Zrot = 5.0f;
  }
  else if (deltaTime >= 75000)
  {
    Xmove = 0.0F;
    Ymove = 0.0F;
    Zrot = 20.0f;
  }
  
  UpdateGait();
  UpdateLegs();
  delay(5);
  
  
  //GetWifiData();
}
