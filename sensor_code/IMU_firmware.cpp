#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <time.h>

//  board addresses for I2C setup
#define ICM_READ  0xD5
#define ICM_WRITE 0xD4

// IMU i2c address
#define IMU_ADDR  0x6A

//  register addresses for accelerometer
#define OUTZ_L_XL 0x2C
#define WHO_AM_I  0x0F
#define OUTZ_H_XL 0x2D
#define X_XL_ADDR 0X28
#define Y_XL_ADDR 0x2A

//  register addresses for gyroscope
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27


// register data for accelerometer & gyro setup
#define XL_CNTRL  0x10
#define G_CNTRL   0x11
#define LOW_POWER 0b00110000
#define HIGH_PFMNC 0b10100000

// setup i2c pins on arduino
#define SDA_PIN A4
#define SCL_PIN A5

//Global Variables
byte RxBuffer[100] = {0};
int RxIdx = 0;
const float GYRO_MULT = 0.004375;

struct IMUData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

unsigned long micros() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long)(ts.tv_sec * 1000000UL + ts.tv_nsec / 1000);
}

void get_events(IMUData* data){
  uint32_t res;
  uint8_t buffer[12];

  res = i2c_smbus_read_block_data(file, OUTX_L_G, 12, &buffer);

  data->gx = (buffer[1] << 8) | buffer[0];
  data->gy = (buffer[3] << 8) | buffer[2];
  data->gz = (buffer[5] << 8) | buffer[4];
  data->ax = (buffer[7] << 8) | buffer[6];
  data->ay = (buffer[9] << 8) | buffer[8];
  data->az = (buffer[11] << 8) | buffer[10];
}

// global variables
double height;
double velocity = 0;
unsigned long prevTime = 0;
double a_ema = 0;
double pitch = 0;
double yaw = 0;
double roll = 0;
IMUData events;

void setup() {
  int file;
  int adapter_nr = 1;

  char filename[20];

  printf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);

  if (file < 0) {
	  printf("Issue with i2c with error: %d", file);
	  exit(1);
  }

  if (ioctl(file, I2C_SLAVE, IMU_ADDR)){
	printf("Issue with i2c error: %d", file);
  }
  
  char buf[12];
  uint32_t res;
  
  // Setup Control Register for Gyro
  res = i2c_smbus_write_word_data(file, G_CNTRL, HIGH_PFMNC);
  if (res < 0){
      printf("gyro control i2c interaction failed");
      exit(1);
  }
  // Setup control register for accel
  res = i2c_smbus_write_word_data(file, XL_CNTRL, HIGH_PFMNC);
  if (res < 0){
      printf("accel control i2c interaction failed");
      exit(1);
  }
  
  // initialize timer module to get dt integrand
  prevTime = micros();
}

void loop() {
  // Get current time
  unsigned long currentTime = micros();
  double dt = (currentTime - prevTime) / 1e6; // in seconds
  prevTime = currentTime;

  get_events(&events);
  // Get raw Z-acceleration
  int16_t z_raw = events.az;

  // Convert raw to m/s² (assuming ±2g FS range)
  double a = z_raw * 0.061 * 9.80665 / 1000.0 - 9.80665;
  double gx = events.gx * GYRO_MULT;
  double gy = events.gy * GYRO_MULT;
  double gz = events.gz * GYRO_MULT;

  // EMA filter
  const float alpha = 0.85;
  a_ema = alpha * a_ema + (1-alpha) * a;
  a = a_ema;

  // ignore xl noise
  if (abs(a) < 0.2){
    // zero velocity correction
    if (abs(a) < 0.03){
      velocity = 0;
    }
    a = 0;
  } 

  // ignore gyro_noise
  if (abs(gx) < 0.1){
    gx = 0;
  }

  if (abs(gy) < 0.1){
    gy = 0;
  }

  if (abs(gz) < 0.1){
    gz = 0;
  }

  //  Integrate gyroscope data
  pitch += gx * dt;
  yaw += gy * dt;
  roll += gz * dt;

  // Integrate acceleration to get velocity
  velocity += a * dt;

  // Integrate velocity to get height
  height += velocity * dt;

  // Output
  printf("Accelerometer (ax, ay, az): %d, %d, %d\n", ax, ay, az);
  printf("Gyroscope (gx, gy, gz): %d, %d, %d\n", gx, gy, gz);
  
}

int main(){
	setup();

	while(1){
		loop();
	}
}

