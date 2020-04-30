// Using the MPU6050 and a simplified Kalman-like algorithm, orientation of the sensor is estimated

#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define PI 3.14159265

#define LSB_PER_G 16384.0       // bits per g
#define LSB_PER_DEGSEC 131      // bits per degree/sec
#define G 9.81

#define ACC_SENSITIVITY 2       // in g
#define GYRO_SENSITIVITY 250    // in degrees/sec

#define LED_PIN 13


/***Variables declaration***/

MPU6050 accelgyro;

float R_Est[3];       // estimation of projection of normalized gravitation vector
float R_Accel[3];     // projection of normalized gravitation vector on x/y/z axis, as measured by accelerometer (in g)
float R_Gyro[3];      // projection of normalized gravitation vector obtained from last estimated value and gyro movement (in g)

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
    Serial.println("Initializing I2C devices...");
    // with accel_range = +-2g, gyro_range = +-250 degrees/sec
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    
    // Update internal offsets (necessary every now and again)
    Serial.println("Updating internal sensor offsets...");
    accelgyro.setXAccelOffset(-3285);
    delay(5);
    accelgyro.setYAccelOffset(1300);
    delay(5);
    accelgyro.setZAccelOffset(1155);
    delay(5);
    accelgyro.setXGyroOffset(108);
    delay(5);
    accelgyro.setYGyroOffset(-68);
    delay(5);
    accelgyro.setZGyroOffset(-2);
    

    t_old = micros();
    firstTime = true;
    
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}


void loop() {


    getEstimation(); 
    
    Serial.print(R_Accel[2]);  //Inclination X axis (as measured by accelerometer)
    Serial.print(",");
    Serial.print(R_Est[2]);    //Inclination X axis (estimated / filtered)
    Serial.println("");
    
}

void getEstimation(){

    static int i;               // for for loops
    static unsigned long t_new; // for time measurement
    static float dt;
    
    // accelerometer and gyro raw values (no units)
    static int16_t a_raw[3];
    static int16_t g_raw[3];
    
    // auxiliary values  
    static float g_ang_vel[3];   // in degrees/sec   
    static float Az[2];          //angles between projection of the vector on XZ/YZ plane and Z axis (deg)
    static float signRzGyro;     // sign of the z component of the gyro estimation for the gravity vector

 
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
        R_Est[i] = R_Accel[i];
      }
      firstTime = false;
    }
    else{
      if(abs(R_Est[2]) < 0.1){
        //Rz is too small and because it is used as reference for computing Axz and Ayz, its error fluctuations will amplify leading to bad results
        //in this case skip the gyro data and just use previous estimate
        for(i=0;i<=2;i++) 
          R_Gyro[i] = R_Est[i];
      }
      else{
        // calculate R_Gyro normally
        
        // get angles between projection of R on ZX/ZY plane and Z axis, based on last R_Est and gyro measurement
        for(i=0;i<=1;i++){
          Az[i] = (atan2(R_Est[i],R_Est[2]) * 180 / PI) + g_ang_vel[i] * dt;  // here is the problem: if Rz too small --> atan() lacks precision in very large numbers      
        }
        
        /*
        // The following doesn't make sense: The gyro measurement is sound, but the estimation comes also from the accelerometer.
        // The accelerometer, uses the gravity vector to compute roll, pitch and yaw. Therefore, when one of the axis alligns with the vector,
        // the rotation around that axis cannot be measured by the accelerometer anymore.

         yaw = (atan2(R_Est[0],R_Est[1]) * 180 / PI) + g_ang_vel[2] * dt;
        */
        
        // estimate sign of RzGyro by looking in what qudrant the angle Axz is: pozitive if Axz in range -90 ..90 => cos(Axz) >= 0
        signRzGyro = ( cos(Az[0] * PI / 180) >=0 ) ? 1 : -1;
  
        // Use analytically derived relationships to obtain the gyro estimation
        R_Gyro[0] = sin(Az[0] * PI / 180) / sqrt( 1 + squared(cos(Az[0] * PI / 180)) * squared(tan(Az[1] * PI / 180)) );  
        R_Gyro[1] = sin(Az[1] * PI / 180) / sqrt( 1 + squared(cos(Az[1] * PI / 180)) * squared(tan(Az[0] * PI / 180)) );
        R_Gyro[2] = signRzGyro * sqrt(1 - squared(R_Gyro[0]) - squared(R_Gyro[1]));
      }

      // normalize gyro estimation
      normalized(R_Gyro);
      
      //combine Accelerometer and Gyro readings
      for(i=0;i<=2;i++) 
        R_Est[i] = (R_Accel[i] + gyro_weight * R_Gyro[i]) / (1 + gyro_weight);

      // normalize estimation vector
      normalized(R_Est);
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
