#include "Arduino.h"
#include "common.h"

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

void BuildRotationMatrix(float x_angle, float y_angle, float z_angle, float *result)
{
  // Convert all angles to radians
  float rad_x_angle = x_angle * DEG_TO_RAD;
  float rad_y_angle = y_angle * DEG_TO_RAD;
  float rad_z_angle = z_angle * DEG_TO_RAD;
  
  // Build rotation matrices 
  float Rx[3][3] = { {1,0,0}, { 0, cosf(rad_x_angle), -sinf(rad_x_angle) }, {0, sinf(rad_x_angle), cosf(rad_x_angle)} };
  float Ry[3][3] = { {cosf(rad_y_angle),0,sinf(rad_y_angle)}, {0,1,0}, {-sinf(rad_y_angle),0,cosf(rad_y_angle)}};
  float Rz[3][3] = { {cosf(rad_z_angle), -sinf(rad_z_angle), 0}, {sinf(rad_z_angle), cosf(rad_z_angle),0}, {0,0,1}};
  // Rxy holds the intermediate result for final Rxyz solution
  float Rxy[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
  // Calculate Rxy
  MatrixMultiply((float*)Rx,(float*)Ry,3,3,3,(float*)Rxy);
  // Calculate Rxyz
  MatrixMultiply((float*)Rxy,(float*)Rz,3,3,3,(float*)result);
}

void RotateTranslate(float *body_points, float x_angle, float y_angle, float z_angle, float x_trans, float y_trans, float z_trans, float *result)
{
  // Build matrix to hold body translate data
  float trans[3][6] = { {x_trans, x_trans, x_trans, x_trans, x_trans, x_trans}, 
                        {y_trans, y_trans, y_trans, y_trans, y_trans, y_trans},
                        {z_trans, z_trans, z_trans, z_trans, z_trans, z_trans} };                      
  float Rxyz[3][3] = {{0,0,0},
                      {0,0,0},
                      {0,0,0}};

  float rot_output[][6] = {{0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0}};
  // Build rotation matrix based off supplied angles                         
  BuildRotationMatrix(x_angle,y_angle,z_angle,(float*)Rxyz);
  
  // Perform rotation on the 6-points of the body
  MatrixMultiply((float*)Rxyz,(float*)body_points,3,3,6,(float*)rot_output);
  
  // Add the body translation values
  MatrixAdd((float*) rot_output, (float*) trans, 3, 6, (float*) result);
}

void BuildTransforms()
{
  float temp[3][3] ={ {0,0,0}, 
                      {0,0,0}, 
                      {0,0,0}};
  // These rotation matrices are to convert from the leg coordinate system to the body coordinate system.                    
  BuildRotationMatrix(0,0,60,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RF_leg_to_body);
  
  BuildRotationMatrix(0,0,0,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RM_leg_to_body);
  
  BuildRotationMatrix(0,0,300,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RR_leg_to_body);
  
  BuildRotationMatrix(0,0,120,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LF_leg_to_body);
  
  BuildRotationMatrix(0,0,180, (float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LM_leg_to_body);
  
  BuildRotationMatrix(0,0,240,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LR_leg_to_body);
  
  
  BuildRotationMatrix(0,0,-60,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RF_body_to_leg);
  
  BuildRotationMatrix(0,0,0,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RM_body_to_leg);
  
  BuildRotationMatrix(0,0,-300,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) RR_body_to_leg);
  
  BuildRotationMatrix(0,0,-120,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LF_body_to_leg);
  
  BuildRotationMatrix(0,0,-180, (float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LM_body_to_leg);
  
  BuildRotationMatrix(0,0,-240,(float*)temp);
  MatrixTranspose((float*) temp,3,3,(float*) LR_body_to_leg);
}

float* GetLegToBodyTransform (byte LegNr)
{
  switch (LegNr)
  {
    case RF:
      return (float*)RF_leg_to_body;
    case RM:
      return (float*)RM_leg_to_body;
    case RR:
      return (float*)RR_leg_to_body;
    case LF:
      return (float*)LF_leg_to_body;
    case LM:
      return (float*)LM_leg_to_body;
    case LR:
      return (float*)LR_leg_to_body;
  }
  return NULL;
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

LegAngles LegIK(float* body_delta_pt, float FeetPosX, float FeetPosY, float FeetPosZ, byte LegNr)
{
  LegAngles result;
  float foot_vector[3][1] = { { FeetPosX }, { FeetPosY }, { FeetPosZ } };
  float body_transform[3][1] = { {0}, {0}, {0} };
  float foot_transform[3][1] = { {0}, {0}, {0} };
  
  MatrixMultiply(GetLegToBodyTransform(LegNr),body_delta_pt,3,3,1,(float*)body_transform);
  MatrixMultiply(GetBodyToLegTransform(LegNr),(float*)foot_vector,3,3,1,(float*)foot_transform);
  
  
  float body_x = body_transform[0][0];  
  float body_y = body_transform[1][0];  
  float body_z = body_transform[2][0];
  
  float foot_x = (CoxaLength + FemurLength) + foot_transform[0][0] - body_x;
  float foot_y = foot_transform[1][0] - body_y;  
  float foot_z = TibiaLength + foot_transform[2][0] - body_z;
  

 
  //float L1 = foot_x - body_x;
  //float Zoffset = body_z - foot_z;
  float L = sqrt( foot_z * foot_z + (foot_x - CoxaLength)*(foot_x - CoxaLength));
  float A1 = acosf(foot_z / L);
  float A2 = acosf( (L*L + FemurLength*FemurLength - TibiaLength*TibiaLength) / (2*L*FemurLength) );
  float A = (A1 + A2) * RAD_TO_DEG;
  float B = acosf( (FemurLength*FemurLength + TibiaLength*TibiaLength - L*L) / (2*FemurLength*TibiaLength) ) * RAD_TO_DEG;

  result.CoxaAngle = atan2f( foot_y , foot_x ) * RAD_TO_DEG;
  result.FemurAngle = A - 90.0F;
  result.TibiaAngle = 90.0F - B;
  return result;
}

void setup() {
  USBSerial.begin(115200);
  HWSerial.begin(115200);

  BuildTransforms();
  InitPositions();
  delay(3000);
  HWSerial.write("#0 PO0 #1 PO-25 #2 PO-50 #4 PO0 #5 PO25 #6 PO50 #8 PO0 #9 PO0 #10 PO50 #16 PO0 #17 PO50 #18 PO0 #20 PO0 #21 PO-50 #22 PO-50 #24 PO0 #25 PO50 #26 PO0\r");
  delay(5000);
}

void InitPositions()
{
  for (int i = 0;i<6;i++)
  {
    Gait[i].x = 0;
    Gait[i].y = 0;
    Gait[i].z = 0;
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
  float body_pts[3][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}};
  float body_delta[3][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}};
  
  //unsigned long time = micros();
  RotateTranslate((float*)body_init,bRot.x,bRot.y,bRot.z,bTrans.x,bTrans.y,bTrans.z, (float*)body_pts);

  MatrixSubtract((float*)body_pts, (float*)body_init,3,6,(float*)body_delta);
  LegAngles angles[6];

  for (int i=0;i<6;i++)
  {
    float body_delta_pt[3][1] = { { body_delta[0][i] }, { body_delta[1][i] }, {body_delta[2][i]} };
    angles[i] = LegIK((float*)body_delta_pt,Gait[i].x,Gait[i].y,Gait[i].z,i);
  }
  //unsigned long dtime = micros() - time;
  //USBSerial.print("Time: ");
  
  //USBSerial.println(dtime);
  
  setLegAngles(angles,300);
}

void setLegAngles(LegAngles *angles, word wMoveTime)
{
  word wCoxaSSCV = 1500;
  word wFemurSSCV = 1500;
  word wTibiaSSCV = 1500;
  
  for (int i = 0;i < 6;i++)
  {
    USBSerial.print("Leg: ");
    USBSerial.print(i);
    USBSerial.print("  ");
    USBSerial.print(angles[i].CoxaAngle);
    USBSerial.print("  ");
    USBSerial.print(angles[i].FemurAngle);
    USBSerial.print("  ");
    USBSerial.println(angles[i].TibiaAngle);
    
    
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
  while (millis() - time <= wMoveTime-1)
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

void loop() {
    
  /*switch (GaitSeq)
  {
    case 0:
      Gait[RF].x = 0.0F;
      Gait[RF].y = -12.5F*2.0;
      Gait[RF].z = 0.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 12.5F*2.0;
      Gait[RM].z = 0.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 0.0F;
      Gait[RR].z = 0.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 6.25F*2.0;
      Gait[LF].z = 0.5F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = -6.25F*2.0;
      Gait[LM].z = 0.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 0.0;
      Gait[LR].z = -30.0F;

      break;
    case 1:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 0.0F;
      Gait[RF].z = -30.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 6.25F*2.0;
      Gait[RM].z = 0.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = -6.25F*2.0;
      Gait[RR].z = 0.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 0.0F;
      Gait[LF].z = 0.0F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = -12.5F*2.0;
      Gait[LM].z = 0.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 12.5F*2.0;
      Gait[LR].z = 0.0F;
      break;
    case 2:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 12.5F*2.0;
      Gait[RF].z = 0.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 0.0F;
      Gait[RM].z = 0.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = -12.5F*2.0;
      Gait[RR].z = 0.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = -6.25F*2.0;
      Gait[LF].z = 0.0F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 0.0F;
      Gait[LM].z = -30.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 6.25F*2.0;
      Gait[LR].z = 0.0F;
      break;
    case 3:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 6.25F*2.0;
      Gait[RF].z = 0.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = -6.25F*2.0;
      Gait[RM].z = 0.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 0.0F;
      Gait[RR].z = -30.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = -12.5F*2.0;
      Gait[LF].z = 0.0F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 12.5F*2.0;
      Gait[LM].z = 0.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = 0.0F;
      Gait[LR].z = 0.0F;
      break;
    case 4:
      Gait[RF].x = 0.0F;
      Gait[RF].y = 0.0F;
      Gait[RF].z = 0.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = -12.5F*2.0;
      Gait[RM].z = 0.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 12.5F*2.0;
      Gait[RR].z = 0.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 0.0F;
      Gait[LF].z = -30.0F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 6.25F*2.0;
      Gait[LM].z = 0.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = -6.25F*2.0;
      Gait[LR].z = 0.0F;
      break;
    case 5:
      Gait[RF].x = 0.0F;
      Gait[RF].y = -6.25F*2.0;
      Gait[RF].z = 0.0F;
      
      Gait[RM].x = 0.0F;
      Gait[RM].y = 0.0F;
      Gait[RM].z = -30.0F;
      
      Gait[RR].x = 0.0F;
      Gait[RR].y = 6.25F*2.0;
      Gait[RR].z = 0.0F;
      
      Gait[LF].x = 0.0F;
      Gait[LF].y = 12.5F*2.0;
      Gait[LF].z = 0.0F;
      
      Gait[LM].x = 0.0F;
      Gait[LM].y = 0.0F;
      Gait[LM].z = 0.0F;
      
      Gait[LR].x = 0.0F;
      Gait[LR].y = -12.5F*2.0;
      Gait[LR].z = 0.0F;
      GaitSeq=-1;
      break;
      
    default:
      GaitSeq=0;
      break;
  }
  GaitSeq++;*/
  //bRot;
  
  
  if (GaitSeq <= 4)
  {
    bTrans.x += 5.0f;
    GaitSeq++;
  }
  else
  {
    GaitSeq = 0;
    bTrans.x = 0;
  }
  
  USBSerial.print("X: ");
  USBSerial.println(bTrans.x);
  //bTrans.x = 50;
  UpdateLegs();
  
  //delay(100);
}
