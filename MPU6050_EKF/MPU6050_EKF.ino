// Using the MPU6050 and an EKF , orientation of the sensor is estimated

// TODO:
// 1) Set the equations of the filter (in all cases, skip the generalization and work with specific dimensions)
// 2) Approximate Jacobian with forward finite differences



#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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

/**********************************************************/
    

void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    // with accel_range = +-2g, gyro_range = +-250 degrees/sec
    accelgyro.initialize();

    // verify connection
//    Serial.println("Testing device connections...");
//    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//
//    
//    // Update internal offsets (necessary every now and again)
//    Serial.println("Updating internal sensor offsets...");
//    accelgyro.setXAccelOffset(-3285);
//    delay(5);
//    accelgyro.setYAccelOffset(1300);
//    delay(5);
//    accelgyro.setZAccelOffset(1155);
//    delay(5);
//    accelgyro.setXGyroOffset(108);
//    delay(5);
//    accelgyro.setYGyroOffset(-68);
//    delay(5);
//    accelgyro.setZGyroOffset(-2);
    

    t_old = micros();
    firstTime = true;

    
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}


void loop() {


    getEstimation(); 

    // Serial.println(no_inverse);

    //Serial.println("Accel Gyro Estimation");
    Serial.print(R_Accel[2]);    //Inclination X axis (as measured by accelerometer)
    Serial.print(" ");
    Serial.print(R_Gyro[2]);    //Inclination X axis (gyro)
    Serial.print(" ");
    Serial.print(R_Estim[2]);   //Inclination X axis (estimated / filtered)
    Serial.println(" ");


//    print_array(F);
    
}

void getEstimation(){

    int i;               // for for loops
    unsigned long t_new; // for time measurement
    
    // accelerometer and gyro raw values (no units)
    int16_t a_raw[3];
    int16_t g_raw[3];
    
    float Temp1[3][3];   // temp arrays for calculations
    float Temp2[3][3];
    float Temp_Vec[3];
    float Temp_Vec2[3];
    
    /***Get measurements and time step***/
    
    t_new = micros(); // in us
    
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&a_raw[0], &a_raw[1], &a_raw[2], &g_raw[0], &g_raw[1], &g_raw[2]);

    dt = (t_new - t_old) / 1000000.0f; // in sec
    t_old = t_new;

//    Serial.print("Raw a/g:\t");
//    print_data(a_raw,3);
//    print_data(g_raw,3);
//    Serial.print('\n');

    /**********************************************************/

    
    /***Get accelerometer data in g and gyro in degrees/sec***/

    for(i=0;i<3;i++){
      R_Accel[i] = a_raw[i] / LSB_PER_G;
      g_ang_vel[i] = g_raw[i] / LSB_PER_DEGSEC;
    }

    // normalize acceleration vector
    normalized(R_Accel);
    
//    Serial.print("Meaningful a/g:\t");
//    print_data(R_Accel,3);
//    print_data(g_ang_vel,3);
//    Serial.print('\n');

    /**********************************************************/

    if(firstTime){
      // initialize algorithm with accelerometer measurement
      for(i=0;i<3;i++){
        R_Estim[i] = R_Accel[i];
      }
      firstTime = false;
    }
    else{

      /** EKF Prediction step **/
      
      if(abs(R_Estim[2]) > 0.1){
        
        // Update with gyro data and previous estimate, only if Rz is not too small.
        // Else, keep the previous estimation (skip prediction step)
        
        // evaluate the update function, using previous estimation and gyro measurement
        evaluate_R(R_Estim,R_Estim);

        // normalize gyro estimation
        normalized(R_Estim);   

         // calculate Jacobian of update function, with f.d.
        update_F();
  
        // update state covariance
        transpose(F,Temp1);      // get the transpose of F
        dot(P, Temp1, Temp2);    // P * F_transpose
        dot(F, Temp2, Temp1);    // F * (P * F_transpose)        
        plus(Temp1, Q, P);       // add process noise      
      }
      else{
        Serial.println("Skipping update!");
      }

      // Save the gyro estimation for comparison
      for(i=0;i<3;i++)
          R_Gyro[i] = R_Estim[i];
      
      /*************/

      
      /** EKF Correction step **/

      // Necessary calculations
      plus(R, P, Temp1);      // innovation covariance in Temp1
      inv(Temp1,Temp2);       // inverse of innovation covariance in Temp2
      dot(P,Temp2,K);         // Kalman gain

      // Correction of state vector
      minus(R_Accel,R_Estim,Temp_Vec); // innovation (residual between measurement and predicted state) in Temp_Vec
      dot(K,Temp_Vec,Temp_Vec2);       // K * innovation in Temp_Vec2  
      plus(R_Estim,Temp_Vec2,R_Estim);  // correct state vector

      // Correction of covariance
      dot(K,P,Temp1);          // K * P in Temp1
      minus(P,Temp1,P);        // correct state covariance
      
      // normalize estimation vector
      normalized(R_Estim);

      /********************************/

      /*     
      // Weighted Method: combine Accelerometer and Gyro readings
      for(i=0;i<=2;i++) 
        R_Estim[i] = (R_Accel[i] + gyro_weight * R_Gyro[i]) / (1 + gyro_weight);
      */
    }
}


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
