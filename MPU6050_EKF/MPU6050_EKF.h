#ifndef TEST_MPU6050_EKF_H
#define TEST_MPU6050_EKF_H

#define DIGITS 4                // number of digits to display for floatting numbers
#define PI 3.14159265

#define LSB_PER_G 16384.0       // bits per g
#define LSB_PER_DEGSEC 131      // bits per degree/sec
#define G 9.81

#define ACC_SENSITIVITY 2       // in g
#define GYRO_SENSITIVITY 250    // in degrees/sec

#define LED_PIN 13


/***Variables declaration***/

MPU6050 accelgyro;

float R_Estim[3];     // estimation of projection of normalized gravitation vector
float R_Accel[3];     // projection of normalized gravitation vector on x/y/z axis, as measured by accelerometer (in g)
float R_Gyro[3];      // projection of normalized gravitation vector obtained from last estimated value and gyro movement (in g)

float P[3][3] = {{1,0,0},{0,1,0},{0,0,1}};                 // state covariance matrix
float Q[3][3] = {{1e-4,0,0},{0,1e-4,0},{0,0,1e-4}};        // model and gyro noise covariance
float R[3][3] = {{1e-3,0,0},{0,1e-3,0},{0,0,1e-3}};        // accelerometer noise covariance

float F[3][3];                                    // update Jacobian (updated in every iteration)
float H[3][3] = {{1,0,0},{0,1,0},{0,0,1}};        // measurement Jacobian
float K[3][3];        // Kalman gain

bool no_inverse = false;

float g_ang_vel[3];     // in degrees/sec
float dt;
float gyro_weight = 10; // the larger, the more we trust the gyro (it's actually w_gyro / w_accel) - fixed
unsigned long t_old;    // for measuring time elapsed
bool firstTime;         // true if it is the first iteration of the estimation algorithm


inline void print_data(const int16_t a[], int len){

    for(int i=0;i<len;i++){
      Serial.print(a[i]); Serial.print("\t");
    }

}

inline void print_data(const float a[], int len){

    for(int i=0;i<len;i++){
      Serial.print(a[i]); Serial.print("\t");
    }

}

inline void print_array(const float A[][3]){
    
    Serial.print("[");
    for(int i=0;i<3;i++){
      
      Serial.print(A[i][0],DIGITS);    
      Serial.print("\t");
      Serial.print(A[i][1],DIGITS);    
      Serial.print("\t");
      Serial.print(A[i][2],DIGITS);    
      Serial.println("\t");
    }
    Serial.println("]");
}

inline float squared(float x){
  return x*x;
}


inline void normalized(float a[]){
  // Normalizes a 3d-vector
  
  float len = sqrt(squared(a[0]) + squared(a[1]) + squared(a[2]));

  for(int i=0;i<3;i++){
    a[i] /= len; 
  }
}

void transpose(float input[][3], float output[][3]){
// Find traspose of a 3x3 matrix

    int i, j;
    
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            output[j][i] = input[i][j];
        }
    }
}

void dot(float A[][3], float B[][3], float result[][3]){
// Multiplication for 3x3 arrays

    int i, j, k;
    
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            result[i][j]=0;
            for(k=0;k<3;k++)
            {
                result[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}


void dot(float A[][3], float B[], float result[]){
// Multiplication of 3x3 array with 3-vector

    int i,j;
    
    for(i=0;i<3;i++)
    {
      result[i]=0;
      for(j=0;j<3;j++)
      {
          result[i] += A[i][j] * B[j];
      }
    }
}


void plus(float A[][3], float B[][3], float result[][3]){
// Addition for 3x3 arrays
  
    int i, j;
    
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
          result[i][j] = A[i][j] + B[i][j];   
        }
    }
}


void minus(float A[][3], float B[][3], float result[][3]){
// Subtraction for 3x3 arrays
  
    int i, j;
    
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
          result[i][j] = A[i][j] - B[i][j];   
        }
    }
}


void plus(float A[], float B[], float result[]){
// Addition for 3-vector
  
    int i;
    
    for(i=0;i<3;i++)
    {
      result[i] = A[i] + B[i];   
    }
}


void minus(float A[], float B[], float result[]){
// Subtraction for 3-vector
  
    int i;
    
    for(i=0;i<3;i++)
    {
      result[i] = A[i] - B[i];   
    }
}


void inv(float A[][3], float result[][3]){
// Finds inverse of 3x3 array

    int i, j;
    float det = 0;

    for(i=0;i<3;i++)
        det = det + (A[0][i]*(A[1][(i+1)%3]*A[2][(i+2)%3] - A[1][(i+2)%3]*A[2][(i+1)%3]));

    if(det == 0.0f)
      no_inverse = true;

    for(i=0;i<3;i++){
        for(j=0;j<3;j++)
            result[i][j] = ((A[(i+1)%3][(j+1)%3] * A[(i+2)%3][(j+2)%3]) - (A[(i+1)%3][(j+2)%3]*A[(i+2)%3][(j+1)%3]))/ det ;
    }
}


void evaluate_R(float R[],float R_result[]){
  
// Evaluates the update function of the EKF, at the given R[n-1] and gives the result in R_result

    int i;
    float Az[2];          //angles between projection of the vector on XZ/YZ plane and Z axis (deg)
    float signRz;         // sign of the z component of the gyro estimation for the gravity vector

    // get angles between projection of R on ZX/ZY plane and Z axis, based on last R_Estim and gyro measurement
    for(i=0;i<=1;i++){
        Az[i] = (atan2(R[i],R[2]) * 180 / PI) + g_ang_vel[i] * dt;
    }

    // estimate sign of RzGyro by looking in what qudrant the angle Axz is: pozitive if Axz in range -90 ..90 => cos(Axz) >= 0
    signRz = ( cos(Az[0] * PI / 180) >=0 ) ? 1 : -1;
        
    R_result[0] = sin(Az[0] * PI / 180) / sqrt( 1 + squared(cos(Az[0] * PI / 180)) * squared(tan(Az[1] * PI / 180)) );
    R_result[1] = sin(Az[1] * PI / 180) / sqrt( 1 + squared(cos(Az[1] * PI / 180)) * squared(tan(Az[0] * PI / 180)) );
    R_result[2] = signRz * sqrt(1 - squared(R_result[0]) - squared(R_result[1]));
}

void update_F(){
// Calculates Jacobian with central finite differences

    float step = 1e-4; // the finite difference step
    int i,j;
    float Rtemp[3],R_step_back[3], R_step_forward[3]; // temporary arrays to store results for finite differences

    
    // for each of the 3 state variables
    for(i=0;i<3;i++){

        // reset Rtemp to R_Estim
        for(j=0;j<3;j++){
            Rtemp[j] = R_Estim[j];
        }
        Rtemp[i] = R_Estim[i] + step;           // forward step only for the state variable under perturbation
        evaluate_R(Rtemp,R_step_forward);       // evaluate update function one step ahead

        
        // reset Rtemp to R_Estim
        for(j=0;j<3;j++){
            Rtemp[j] = R_Estim[j];
        }
        Rtemp[i] = R_Estim[i] - step;           // backward step only for the state variable under perturbation
        evaluate_R(Rtemp,R_step_back);          // evaluate update function one step ahead
        

        // fill the corresponding column of J
        for(j=0;j<3;j++){
            F[j][i] = (R_step_forward[j] - R_step_back[j]) / (2*step);
        }     

    }
}

#endif //TEST_MPU6050_EKF_H
