/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <math.h>         // Math library required for 'sqrt'
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc/mpu.h>
#include <rc/time.h>
#include <inttypes.h>    // for timing

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

static int enable_warnings = 0;

// System constants
//#define deltat 0.010f    // sampling period in seconds (shown as 10 ms)
#define gyroMeasError 3.14159265358979f * (0.2f / 180.0f) // gyro measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979f * (0.0f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta (float)sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta (float)sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta

// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float m_x, m_y, m_z; // magnetometer measurements

float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

// function declarations
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float *roll, float *pitch, float *yaw, double deltat);

// timing
uint64_t a,b,nanos;
double deltat = 0.010; // sampling period in seconds (initialized at 10 ms)

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	rc_mpu_data_t data; //struct to hold new data

	// use defaults for now, except also enable magnetometer.
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.enable_magnetometer = 1;
        conf.show_warnings = enable_warnings;
        if(rc_mpu_initialize(&data, conf)){
                fprintf(stderr,"rc_mpu_initialize_failed\n");
                return -1;
        }

	float roll, pitch, yaw; // euler angle representation of orientation

	printf("   Accel XYZ(m/s^2)  |");
	printf("   Gyro XYZ (rad/s)  |");
	printf("  Mag Field XYZ(uT)  |");
	printf("      R/P/Y (deg)    |");
	printf(" Ts (ms) |");
	printf("\n");

	int timing = 0;

	// run
	rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
        if(rc_get_state()==RUNNING){
			printf("\r");

			// read sensor data
        	if(rc_mpu_read_accel(&data)<0)
                printf("read accel data failed\n");
            if(rc_mpu_read_gyro(&data)<0)
               	printf("read gyro data failed\n");
            if(rc_mpu_read_mag(&data))
                printf("read mag data failed\n");

			a_x = data.accel[0];
			a_y = data.accel[1];
			a_z = data.accel[2];

			w_x = data.gyro[0]*DEG_TO_RAD;
			w_y = data.gyro[1]*DEG_TO_RAD;
			w_z = data.gyro[2]*DEG_TO_RAD;

			m_x = data.mag[0];
			m_y = data.mag[1];
			m_z = data.mag[2];
			
			printf("%6.2f %6.2f %6.2f |", a_x, a_y, a_z);
			printf("%6.1f %6.1f %6.1f |", w_x, w_y, w_z);
			printf("%6.1f %6.1f %6.1f |", m_x, m_y, m_z);

			if(timing == 1) {
				b      = rc_nanos_since_epoch(); //end time
				nanos  = b - a;
				deltat = (double)nanos/1000000000;
			}

			filterUpdate(w_x, w_y, w_z, a_x, a_y, a_z, m_x, m_y, m_z, &roll, &pitch, &yaw, deltat);
		
			a = rc_nanos_since_epoch(); //start time
		
			if(timing == 0)
				timing = 1;

			roll  =  roll*(180.0f/3.14159265358979f);
			pitch = pitch*(180.0f/3.14159265358979f);
			yaw   =   yaw*(180.0f/3.14159265358979f);

			printf("%6.2f %6.2f %6.2f |", roll, pitch, yaw);

	        //b = rc_nanos_since_epoch(); //end time
			printf(" %" PRIu64 "ms  |", nanos/1000000);

			// always sleep at some point
        	rc_usleep(10000);
		}
	}

// close file descriptors
rc_remove_pid_file();	// remove pid file LAST
rc_mpu_power_off();
return 0;
}

/**
 * Update the 6DoF Madgwick Filter
 */
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float *roll, float *pitch, float *yaw, double deltat)
{
// local system variables
float norm; // vector norm
float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
float h_x, h_y, h_z; // computed flux in the earth frame
// axulirary variables to avoid reapeated calcualtions
float halfSEq_1 = 0.5f * SEq_1;
float halfSEq_2 = 0.5f * SEq_2;
float halfSEq_3 = 0.5f * SEq_3;
float halfSEq_4 = 0.5f * SEq_4;
float twoSEq_1 = 2.0f * SEq_1;
float twoSEq_2 = 2.0f * SEq_2;
float twoSEq_3 = 2.0f * SEq_3;
float twoSEq_4 = 2.0f * SEq_4;
float twob_x = 2.0f * b_x;
float twob_z = 2.0f * b_z;
float twob_xSEq_1 = 2.0f * b_x * SEq_1;
float twob_xSEq_2 = 2.0f * b_x * SEq_2;
float twob_xSEq_3 = 2.0f * b_x * SEq_3;
float twob_xSEq_4 = 2.0f * b_x * SEq_4;
float twob_zSEq_1 = 2.0f * b_z * SEq_1;
float twob_zSEq_2 = 2.0f * b_z * SEq_2;
float twob_zSEq_3 = 2.0f * b_z * SEq_3;
float twob_zSEq_4 = 2.0f * b_z * SEq_4;
float SEq_1SEq_2;
float SEq_1SEq_3 = SEq_1 * SEq_3;
float SEq_1SEq_4;
float SEq_2SEq_3;
float SEq_2SEq_4 = SEq_2 * SEq_4;
float SEq_3SEq_4;
float twom_x = 2.0f * m_x;
float twom_y = 2.0f * m_y;
float twom_z = 2.0f * m_z;
// normalise the accelerometer measurement
norm = (float)sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
a_x /= norm;
a_y /= norm;
a_z /= norm;
// normalise the magnetometer measurement
norm = (float)sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
m_x /= norm;
m_y /= norm;
m_z /= norm;
// compute the objective function and Jacobian
f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
J_12or23 = 2.0f * SEq_4;
J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
J_14or21 = twoSEq_2;
J_32 = 2.0f * J_14or21; // negated in matrix multiplication
J_33 = 2.0f * J_11or24; // negated in matrix multiplication
J_41 = twob_zSEq_3; // negated in matrix multiplication
J_42 = twob_zSEq_4;
J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
J_52 = twob_xSEq_3 + twob_zSEq_1;
J_53 = twob_xSEq_2 + twob_zSEq_4;
J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
J_61 = twob_xSEq_3;
J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
J_64 = twob_xSEq_2;
// compute the gradient (matrix multiplication)
SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
// normalise the gradient to estimate direction of the gyroscope error
norm = (float)sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
SEqHatDot_1 = SEqHatDot_1 / norm;
SEqHatDot_2 = SEqHatDot_2 / norm;
SEqHatDot_3 = SEqHatDot_3 / norm;
SEqHatDot_4 = SEqHatDot_4 / norm;
// compute angular estimated direction of the gyroscope error
w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
// compute and remove the gyroscope baises
w_bx += w_err_x * deltat * zeta;
w_by += w_err_y * deltat * zeta;
w_bz += w_err_z * deltat * zeta;
w_x -= w_bx;
w_y -= w_by;
w_z -= w_bz;
// compute the quaternion rate measured by gyroscopes
SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
// compute then integrate the estimated quaternion rate
SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
// normalise quaternion
norm = (float)sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
SEq_1 /= norm;
SEq_2 /= norm;
SEq_3 /= norm;
SEq_4 /= norm;
// compute flux in the earth frame
SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
SEq_1SEq_3 = SEq_1 * SEq_3;
SEq_1SEq_4 = SEq_1 * SEq_4;
SEq_3SEq_4 = SEq_3 * SEq_4;
SEq_2SEq_3 = SEq_2 * SEq_3;
SEq_2SEq_4 = SEq_2 * SEq_4;
h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
// normalise the flux vector to have only components in the x and z
b_x = (float)sqrt((h_x * h_x) + (h_y * h_y));
b_z = h_z;

// Convert to Euler Angles
*roll  = atan2(2*SEq_3*SEq_4 - 2*SEq_1*SEq_2, 2*SEq_1*SEq_1 + 2*SEq_4*SEq_4 - 1);
*pitch = -asin(2*SEq_2*SEq_4 + 2*SEq_1*SEq_3);
*yaw   = atan2(2*SEq_2*SEq_3 - 2*SEq_1*SEq_4, 2*SEq_1*SEq_1 + 2*SEq_2*SEq_2 - 1);
}
