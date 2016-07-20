#include "Arduino.h"
#include "common.h"

#include <string.h>
#include <SPI.h>

uint32_t startTime;

void MatrixPrint(float* A, int m, int n, char* label){
     // A = input matrix (m x n)
     int i,j;
     USBSerial.println(label);
     USBSerial.println(" Matrix values:");
     for (i=0; i<m; i++){
           for (j=0;j<n;j++){
                 USBSerial.print(A[n*i+j]);
                 USBSerial.print("; i=");
                 USBSerial.print(i);
                 USBSerial.print(" j=");
                 USBSerial.println(j);
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
  
    SSC32Serial.print("#");
    SSC32Serial.print(pgm_read_byte(&CoxaPin[i]), DEC);
    SSC32Serial.print("P");
    SSC32Serial.print(wCoxaSSCV, DEC);
    SSC32Serial.print("#");
    SSC32Serial.print(pgm_read_byte(&FemurPin[i]), DEC);
    SSC32Serial.print("P");
    SSC32Serial.print(wFemurSSCV, DEC);
    SSC32Serial.print("#");
    SSC32Serial.print(pgm_read_byte(&TibiaPin[i]), DEC);
    SSC32Serial.print("P");
    SSC32Serial.print(wTibiaSSCV, DEC);
  }
  //USBSerial.println("");
  SSC32Serial.print("T");
  SSC32Serial.println(wMoveTime, DEC);
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
    SSC32Serial.write("Q \r");
    delay(1);
    if (SSC32Serial.available() > 0) 
    {
      char incomingByte = SSC32Serial.read();
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
      Gait[RR].z = -LiftHeight;
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

void GetBlueToothData()
{
  int rd = BlueToothSerial.available();
  if (rd > 0) 
  {
    byte tmp_buffer[80];
    int n = BlueToothSerial.readBytes((byte *)tmp_buffer, rd);
    USBSerial.println(F("Getting Data"));
    //ProcessBuffer((byte *)cmd_buffer,n);
    for (int i =0;i<n;i++)
    {
      if (tmp_buffer[i] != '\n')
      {
        cmd_buffer[cmd_buffer_ptr++] = tmp_buffer[i];
      }
      else
      {
        USBSerial.println(F("Process"));
        ProcessBuffer();
      }
    }
  }
}

float getFloatBuffer(int start_pos)
{
  float2bytes f2b;
  f2b.b[0] = cmd_buffer[start_pos+3];
  //Serial.println(f2b.b[0],HEX);
  f2b.b[1] = cmd_buffer[start_pos+2];
  //Serial.println(f2b.b[1],HEX);
  f2b.b[2] = cmd_buffer[start_pos+1];
  //Serial.println(f2b.b[2],HEX);
  f2b.b[3] = cmd_buffer[start_pos];
  ///Serial.println(f2b.b[3],HEX);
  //Serial.println(f2b.f,4);
  return f2b.f;
  /*float f;
  uint8_t *p = (uint8_t*)&f;
  for (int i = 0;i<4;i++)
  {
    p[i] = cmd_buffer[start_pos+i];
  }
  return f;*/
}

void ProcessBuffer()
{
  byte cmd_byte = cmd_buffer[0];
  switch (cmd_byte)
  {
    case '%':
      Serial.println(F("BlueTooth Cmd"));
      break;
    case 'R':
      Serial.println(F("Setting Rot"));
      bRot.x = getFloatBuffer(1);
      bRot.y = getFloatBuffer(5);
      bRot.z = getFloatBuffer(9);
      break;
    case 'T':
      Serial.println(F("Setting Trans"));
      bTrans.x = getFloatBuffer(1);
      bTrans.y = getFloatBuffer(5);
      bTrans.z = getFloatBuffer(9);
      break;
    case 'G':
      Serial.println(F("Setting Gait"));
      Xmove = getFloatBuffer(1);
      //Serial.println(Xmove,2 );
      Ymove = getFloatBuffer(5);
      //Serial.println(Ymove,2);
      Zrot = getFloatBuffer(9);
      //Serial.println(Zrot,2);
      LiftHeight = getFloatBuffer(13);
      Serial.println(LiftHeight,2);
      break;
  }
  InitializeCommandBuffer();
}

void BlueToothSetup()
{
  BlueToothSerial.print("$$$");
  delay(400);
  BlueToothSerial.print("S~,5\r");
  delay(400);
  BlueToothSerial.print("SN,LINK\r");
  delay(400);
  BlueToothSerial.print("SC,0024\r");
  delay(400);
  BlueToothSerial.print("SD,0704\r");
  delay(400);
  BlueToothSerial.print("ST,253\r");
  delay(400);
  BlueToothSerial.print("SO,%\r");
  delay(400);
  BlueToothSerial.print("T,1\r");
  delay(400);
  BlueToothSerial.print("SM,0\r");
  delay(400);
  BlueToothSerial.print("SA,2\r");
  delay(400);
  BlueToothSerial.print("---\r");
}
 
void setup() {
  USBSerial.begin(115200);
  SSC32Serial.begin(115200);
  BlueToothSerial.begin(115200);
  delay(1000);
  //BlueToothSetup();
  BuildTransforms();
  InitializePositions();
  InitializeCommandBuffer();
  SSC32Serial.write("#0 PO0 #1 PO-25 #2 PO-50 #4 PO0 #5 PO25 #6 PO50 #8 PO0 #9 PO0 #10 PO50 #16 PO0 #17 PO50 #18 PO0 #20 PO0 #21 PO-50 #22 PO-50 #24 PO0 #25 PO50 #26 PO0\r");
  delay(5000);
  startTime = millis();
  USBSerial.println("Robot Start");

}

void loop() 
{
  /*uint32_t deltaTime = millis() - startTime;
  if (deltaTime < 10000)
  {
    Xmove = 0.0F;
    Ymove = 50.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 10000 && deltaTime < 20000)
  {
    Xmove = 50.0F;
    Ymove = 50.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 20000 && deltaTime < 30000)
  {
    Xmove = -50.0F;
    Ymove = -50.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 30000 && deltaTime < 40000)
  {
    Xmove = 50.0F;
    Ymove = 0.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 40000 && deltaTime < 50000)
  {
    Xmove = -50.0F;
    Ymove = 0.0F;
    Zrot = 0;
  }
  else if (deltaTime >= 50000 && deltaTime < 75000)
  {
    Xmove = 0.0F;
    Ymove = 50.0F;
    Zrot = 5.0f;
  }
  else if (deltaTime >= 75000)
  {
    Xmove = 0.0F;
    Ymove = 0.0F;
    Zrot = 20.0f;
  }*/
  GetBlueToothData();
  UpdateGait();
  UpdateLegs();
  //delay(5);
}
