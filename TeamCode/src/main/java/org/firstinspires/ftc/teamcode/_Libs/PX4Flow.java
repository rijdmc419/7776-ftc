
/* ============================================
 NavX-MXP and NavX-Micro source code is placed under the MIT license
 Copyright (c) 2015 Kauai Labs

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */


/**
 * The PX4Flow class provides an interface to PX4Flow optical flow camera capabilities
 * via I2C on the Android-based FTC robotics control system, where communications occur via the
 * "Core Device Interface Module" produced by Modern Robotics, inc.
 */

// As described in the documentation
// http://pixhawk.org/modules/px4flow

package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.robotcore.hardware.I2cDevice;

import android.os.Process;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.Arrays;

class px4_frame
{
    public short frame_count;// counts created I2C frames
    public short pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
    public short pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
    public short flow_comp_m_x;// x velocity*1000 in meters / timestep
    public short flow_comp_m_y;// y velocity*1000 in meters / timestep
    public short qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    public short gyro_x_rate; //gyro x rate
    public short gyro_y_rate; //gyro y rate
    public short gyro_z_rate; //gyro z rate
    public byte  gyro_range; // gyro range
    public byte  sonar_timestamp;// timestep in milliseconds between I2C frames
    public short ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} 

class px4_integral_frame
{
    public short frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    public short pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    public short pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    public short gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    public short gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    public short gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    public int   integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    public int   sonar_timestamp;// time since last sonar update [microseconds]
    public short ground_distance;// Ground distance in meters*1000 [meters*1000]
    public short gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    public byte  quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} 

class PX4Flow
{

    // 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2) 
    final byte PX4FLOW_ADDRESS = 0x42;

    // timeout in milliseconds for PX4Flow read
    final byte PX4FLOW_TIMEOUT = 10;

    // If set to true, will print error messages on Serial
    final boolean PX4FLOW_DEBUG = true;

    private px4_frame frame;
    private px4_integral_frame iframe;

/*
    public PX4Flow()
    {
       frame = new px4_frame;
       iframe = new px4_integral_frame;
    }

    I2cDevice mDevice;

    public boolean update()
    {
        //send 0x0 to PX4FLOW module and receive back 22 Bytes data
        Wire.beginTransmission(PX4FLOW_ADDRESS);
        Wire.write(0x0);
        Wire.endTransmission();

        // request 22 bytes from the module
        Wire.requestFrom(PX4FLOW_ADDRESS, 22);
    }

    public boolean update_integral()
    {
        //send 0x16 to PX4FLOW module and receive back 25 Bytes data
        Wire.beginTransmission(PX4FLOW_ADDRESS);
        Wire.write(0x16);
        Wire.endTransmission();

        // request 25 bytes from the module
        Wire.requestFrom(PX4FLOW_ADDRESS, 26);
    }
*/

    // Simple frame
    public short frame_count() {
  return frame.frame_count;
}
    public short pixel_flow_x_sum() {
  return frame.pixel_flow_x_sum;
}
    public short pixel_flow_y_sum() {
  return frame.pixel_flow_y_sum;
}
    public short flow_comp_m_x() {
  return frame.flow_comp_m_x;
}
    public short flow_comp_m_y() {
  return frame.flow_comp_m_y;
}
    public short gyro_x_rate() {
  return frame.gyro_x_rate;
}
    public short gyro_y_rate() {
  return frame.gyro_y_rate;
}
    public short gyro_z_rate() {
  return frame.gyro_z_rate;
}
    public short qual() {
  return frame.qual;
}
    public byte sonar_timestamp() {
  return frame.sonar_timestamp;
}
    public short ground_distance() {
  return frame.ground_distance;
}

    // Integral frame
    public short frame_count_since_last_readout() {
  return iframe.frame_count_since_last_readout;
}
    public short pixel_flow_x_integral() {
  return iframe.pixel_flow_x_integral;
}
    public short pixel_flow_y_integral() {
  return iframe.pixel_flow_y_integral;
}
    public short gyro_x_rate_integral() {
  return iframe.gyro_x_rate_integral;
}
    public short gyro_y_rate_integral() {
  return iframe.gyro_y_rate_integral;
}
    public short gyro_z_rate_integral() {
  return iframe.gyro_z_rate_integral;
}
    public int integration_timespan() {
  return iframe.integration_timespan;
}
    public int sonar_timestamp_integral() {
  return iframe.sonar_timestamp;
}
    public short ground_distance_integral() {
  return iframe.ground_distance;
}
    public short gyro_temperature() {
  return iframe.gyro_temperature;
}
    public byte quality_integral() {
  return iframe.quality;
}



    /**
     * The DeviceDataType specifies the
     * type of data to be retrieved from the sensor.  Due to limitations in the
     * communication bandwidth, only a subset of all available data can be streamed
     * and still maintain a 50Hz update rate via the Core Device Interface Module,
     * since it is limited to a maximum of one 26-byte transfer every 10ms.
     * Note that if all data types are required,
     */
    public enum DeviceDataType {
        /**
         * (default):  simple latest sample data
         */
        kSimpleFrame(1),
        /**
         * integral data
         */
        kIntegralFrame(2),
        /**
         * All data.  Note that on a
         * Android-based FTC robot using the "Core Device Interface Module", acquiring
         * all data requires two I2C transactions.
         */
        kAll(3);

        private int value;

        private DeviceDataType(int value){
            this.value = value;
        }

        public int getValue(){
            return this.value;
        }
    };

    interface IoCallback {
        public boolean ioComplete( boolean read, int address, int len, byte[] data);
    };


    private static PX4Flow instance = null;
    private static boolean enable_logging = false;
    private static final int NAVX_DEFAULT_UPDATE_RATE_HZ = 50;

    private DeviceInterfaceModule dim = null;
    private navXIOThread io_thread_obj;
    private Thread io_thread;
    private int update_rate_hz = NAVX_DEFAULT_UPDATE_RATE_HZ;
    private IDataArrivalSubscriber callbacks[];
    private final int MAX_NUM_CALLBACKS = 3;

    AHRSProtocol.AHRSPosUpdate curr_data;
    BoardState board_state;
    AHRSProtocol.BoardID board_id;
    IMUProtocol.GyroUpdate raw_data_update;


    protected AHRS(DeviceInterfaceModule dim, int dim_i2c_port,
                   DeviceDataType data_type, int update_rate_hz) {
        this.callbacks = new IDataArrivalSubscriber[MAX_NUM_CALLBACKS];
        this.dim = dim;
        this.update_rate_hz = update_rate_hz;
        this.curr_data = new AHRSProtocol.AHRSPosUpdate();
        this.board_state = new BoardState();
        this.board_id = new AHRSProtocol.BoardID();
        this.raw_data_update = new IMUProtocol.GyroUpdate();

        io_thread_obj   = new navXIOThread(dim_i2c_port, update_rate_hz, data_type, curr_data);
        io_thread_obj.start();

        io_thread       = new Thread(io_thread_obj);
        io_thread.start();
    }

    /**
     * Registers a callback interface.  This interface
     * will be called back when new data is available,
     * based upon a change in the sensor timestamp.
     *<p>
     * Note that this callback will occur within the context of the
     * device IO thread, which is not the same thread context the
     * caller typically executes in.
     */
    public boolean registerCallback( IDataArrivalSubscriber callback ) {
        boolean registered = false;
        for ( int i = 0; i < this.callbacks.length; i++ ) {
            if (this.callbacks[i] == null) {
                this.callbacks[i] = callback;
                registered = true;
                break;
            }
        }
        return registered;
    }

    /**
     * Deregisters a previously registered callback interface.
     *
     * Be sure to deregister any callback which have been
     * previously registered, to ensure that the object
     * implementing the callback interface does not continue
     * to be accessed when no longer necessary.
     */
    public boolean deregisterCallback( IDataArrivalSubscriber callback ) {
        boolean deregistered = false;
        for ( int i = 0; i < this.callbacks.length; i++ ) {
            if (this.callbacks[i] == callback) {
                this.callbacks[i] = null;
                deregistered = true;
                break;
            }
        }
        return deregistered;
    }

    public void close() {
        io_thread_obj.stop();
        for ( int i = 0; i < callbacks.length; i++ ) {
            callbacks[i] = null;
        }
        try {
            io_thread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        instance = null;
    }

    /**
     * Returns the single instance (singleton) of the AHRS class.  If the singleton
     * does not alrady exist, it will be created using the parameters passed in.
     * The default update rate will be used.
     *
     * If the singleton already exists, the parameters passed in will be ignored.
     * @return The singleton AHRS class instance.
     */
    public static AHRS getInstance(DeviceInterfaceModule dim, int dim_i2c_port,
                                   DeviceDataType data_type) {
        if (instance == null) {
            instance = new AHRS(dim, dim_i2c_port, data_type, NAVX_DEFAULT_UPDATE_RATE_HZ);
        }
        return instance;
    }

    /**
     * Returns the single instance (singleton) of the AHRS class.  If the singleton
     * does not alrady exist, it will be created using the parameters passed in,
     * including a custom update rate.  Use this function if an update rate other than
     * the default is desired.
     *
     * NOTE:  The range of valid requested update rates is from 4 to 66.  However, the
     * actual update does not always match the requested update rate.  The following table
     * summarizes the requested to actual update rate lookup table:
     *
     *   Actual       Requested
     *   66.6         58-66
     *   50           45-57
     *   40           37-44
     *   33.3         31-36
     *   28.57        27-30
     *   25           25-26
     *   22.22        22-23
     *   20           20-21
     *   18.18        18-19
     *   16.67        17
     *   15.38        15-16
     *   14.28        14
     *
     * Requested values below 14Hz result in an actual rate which is accurate to within 1Hz.
     *
     * The reason for this difference is that the update rate must be an even multiple of
     * a 200Hz clock (5ms).  So an actual of 66.6 (15ms/sample) can be calculated as follows:
     *      actual_rate = 200 / (200 / requested_rate)
     *
     * The getActualUpdateRate() can be used to calculate this value.
     *
     * @return The singleton AHRS class instance.
     */
    public static AHRS getInstance(DeviceInterfaceModule dim, int dim_i2c_port,
                                   DeviceDataType data_type, byte update_rate_hz) {
        if (instance == null) {
            instance = new AHRS(dim, dim_i2c_port, data_type, update_rate_hz);
        }
        return instance;
    }

    /* Configures the AHRS class logging.  To enable logging,
     * the input parameter should be set to 'true'.
     */
    public static void setLogging( boolean enabled ) {
        enable_logging = enabled;
    }

    /* Returns 'true' if AHRS class logging is enabled, otherwise
     * returns 'false'.
     */
    public static boolean getLogging() {
        return enable_logging;
    }

    /* Returns the "port number" on the Core Device Interface Module
       in which the navX-Model device is connected.
     */
    public int getDimI2cPort() {
        return this.io_thread_obj.dim_port;
    }

    /* Returns the currently configured DeviceDataType.
     */
    public DeviceDataType getDeviceDataType() {
        return this.io_thread_obj.data_type;
    }


    /**
     * Indicates whether the sensor is currently connected
     * to the host computer.  A connection is considered established
     * whenever communication with the sensor has occurred recently.
     *<p>
     * @return Returns true if a valid update has been recently received
     * from the sensor.
     */

    public boolean isConnected() {
        return io_thread_obj.isConnected();
    }

    /**
     * Returns the navX-Model device's currently configured update
     * rate.  Note that the update rate that can actually be realized
     * is a value evenly divisible by the navX-Model device's internal
     * motion processor sample clock (200Hz).  Therefore, the rate that
     * is returned may be lower than the requested sample rate.
     *
     * The actual sample rate is rounded down to the nearest integer
     * that is divisible by the number of Digital Motion Processor clock
     * ticks.  For instance, a request for 58 Hertz will result in
     * an actual rate of 66Hz (200 / (200 / 58), using integer
     * math.
     *
     * @return Returns the current actual update rate in Hz
     * (cycles per second).
     */

    public byte getActualUpdateRate() {
        final int NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ = 200;
        int integer_update_rate =  board_state.update_rate_hz;
        int realized_update_rate = NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ /
                (NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ / integer_update_rate);
        return (byte)realized_update_rate;
    }

    /**
     * Returns the current number of data samples being transferred
     * from the navX-Model device in the last second.  Note that this
     * number may be greater than the sensors update rate.
     *
     * @return Returns the count of data samples received in the
     * last second.
     */

    public int getCurrentTransferRate() {
        return this.io_thread_obj.last_second_hertz;
    }

    /* Returns the number of navX-Model processed data samples
     * that were retrieved from the sensor that had a sensor
     * timestamp value equal to the timestamp of the last
     * sensor data sample.
     *
     * This information can be used to match the navX-Model
     * sensor update rate w/the effective update rate which
     * can be achieved in the Robot Controller, taking into
     * account the communication over the network with the
     * Core Device Interface Module.
     */

    public int getDuplicateDataCount() {
        return this.io_thread_obj.getDuplicateDataCount();
    }

    /**
     * Returns the count in bytes of data received from the
     * sensor.  This could can be useful for diagnosing
     * connectivity issues.
     *<p>
     * If the byte count is increasing, but the update count
     * (see getUpdateCount()) is not, this indicates a software
     * misconfiguration.
     * @return The number of bytes received from the sensor.
     */
    public double getByteCount() {
        return io_thread_obj.getByteCount();
    }

    /**
     * Returns the count of valid updates which have
     * been received from the sensor.  This count should increase
     * at the same rate indicated by the configured update rate.
     * @return The number of valid updates received from the sensor.
     */
    public double getUpdateCount() {
        return io_thread_obj.getUpdateCount();
    }

    /**
     * Returns the current linear acceleration in the X-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the x-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the X-axis (in G).
     */
    public float getWorldLinearAccelX()
    {
        return curr_data.linear_accel_x;
    }

    /**
     * Returns the current linear acceleration in the Y-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the Y-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the Y-axis (in G).
     */
    public float getWorldLinearAccelY()
    {
        return curr_data.linear_accel_y;
    }

    /**
     * Returns the current linear acceleration in the Z-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the Z-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the Z-axis (in G).
     */
    public float getWorldLinearAccelZ()
    {
        return curr_data.linear_accel_z;
    }


    /**
     * Returns the "fused" (9-axis) heading.
     *<p>
     * The 9-axis heading is the fusion of the yaw angle, the tilt-corrected
     * compass heading, and magnetic disturbance detection.  Note that the
     * magnetometer calibration procedure is required in order to
     * achieve valid 9-axis headings.
     *<p>
     * The 9-axis Heading represents the sensor's best estimate of current heading,
     * based upon the last known valid Compass Angle, and updated by the change in the
     * Yaw Angle since the last known valid Compass Angle.  The last known valid Compass
     * Angle is updated whenever a Calibrated Compass Angle is read and the sensor
     * has recently rotated less than the Compass Noise Bandwidth (~2 degrees).
     * @return Fused Heading in Degrees (range 0-360)
     */
    public float getFusedHeading()
    {
        return curr_data.fused_heading;
    }


    /**
     * Returns the current raw (unprocessed) X-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the X Axis is referred to as "Pitch".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getPitch()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroX() {
        return this.raw_data_update.gyro_x / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) Y-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the T Axis is referred to as "Roll".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getRoll()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroY() {
        return this.raw_data_update.gyro_y / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) Z-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the T Axis is referred to as "Yaw".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getYaw()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroZ() {
        return this.raw_data_update.gyro_z / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) X-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * X axis acceleration data is accessible via the {@link #getWorldLinearAccelX()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelX() {
        return this.raw_data_update.accel_x / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    /**
     * Returns the current raw (unprocessed) Y-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * Y axis acceleration data is accessible via the {@link #getWorldLinearAccelY()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelY() {
        return this.raw_data_update.accel_y / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    /**
     * Returns the current raw (unprocessed) Z-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * Z axis acceleration data is accessible via the {@link #getWorldLinearAccelZ()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelZ() {
        return this.raw_data_update.accel_z / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    private final float UTESLA_PER_DEV_UNIT = 0.15f;

    /**
     * Returns the current raw (unprocessed) X-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagX() {
        return this.raw_data_update.mag_x / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current raw (unprocessed) Y-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagY() {
        return this.raw_data_update.mag_y / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current raw (unprocessed) Z-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagZ() {
        return this.raw_data_update.mag_z / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current barometric pressure (in millibar) [navX Aero only].
     *<p>
     *This value is valid only if a barometric pressure sensor is onboard.
     *
     * @return Returns the current barometric pressure (in millibar).
     */
    public float getPressure() {
        // TODO implement for navX-Aero.
        return 0;
    }

    class navXIOThread implements Runnable, AHRS.IoCallback {

        int dim_port;
        int update_rate_hz;
        protected boolean keep_running;
        boolean request_zero_yaw;
        boolean is_connected;
        int byte_count;
        int update_count;
        DeviceDataType data_type;
        AHRSProtocol.AHRSPosUpdate ahrspos_update;
        long curr_sensor_timestamp;
        boolean cancel_all_reads;
        boolean first_bank;
        long last_valid_sensor_timestamp;
        int duplicate_sensor_data_count;
        int last_second_hertz;
        int hertz_counter;
        Object io_thread_event;
        Object reset_yaw_critical_section;

        final int NAVX_REGISTER_FIRST           = IMURegisters.NAVX_REG_WHOAMI;
        final int NAVX_REGISTER_PROC_FIRST      = IMURegisters.NAVX_REG_SENSOR_STATUS_L;
        final int NAVX_REGISTER_RAW_FIRST       = IMURegisters.NAVX_REG_QUAT_W_L;
        final int I2C_TIMEOUT_MS                = 500;

        public navXIOThread( int port, int update_rate_hz, DeviceDataType data_type,
                             AHRSProtocol.AHRSPosUpdate ahrspos_update) {
            this.dim_port = port;
            this.keep_running = false;
            this.update_rate_hz = update_rate_hz;
            this.request_zero_yaw = false;
            this.is_connected = false;
            this.byte_count = 0;
            this.update_count = 0;
            this.ahrspos_update = ahrspos_update;
            this.data_type = data_type;
            this.cancel_all_reads = false;
            this.first_bank = true;
            this.last_valid_sensor_timestamp = 0;
            this.duplicate_sensor_data_count = 0;
            this.last_second_hertz = 0;
            this.hertz_counter = 0;
            this.io_thread_event = new Object();
            this.reset_yaw_critical_section = new Object();

            android.os.Process.setThreadPriority(Process.THREAD_PRIORITY_MORE_FAVORABLE);
        }

        public void start() {
            keep_running = true;
        }
        public void stop() {
            keep_running = false;
            signalThread();
        }

        public void zeroYaw() {
            synchronized(reset_yaw_critical_section) {
                request_zero_yaw = true;
                /* Notify all data subscribers that the yaw
                   is about to be reset.
                 */
                for ( int i = 0; i < callbacks.length; i++ ) {
                    IDataArrivalSubscriber callback = callbacks[i];
                    if (callback != null) {
                        callback.yawReset();
                    }
                }
            }
            signalThread();
        }

        public int getByteCount() {
            return byte_count;
        }

        public int getDuplicateDataCount() {
            return duplicate_sensor_data_count;
        }

        public void addToByteCount( int new_byte_count ) {
            byte_count += new_byte_count;
        }

        public int getUpdateCount() {
            return update_count;
        }

        public void incrementUpdateCount() {
            update_count++;
        }

        public boolean isConnected() {
            return is_connected;
        }

        public void setConnected( boolean new_connected ) {
            is_connected = new_connected;
            if ( !is_connected ) {
                signalThread();
            }
        }

        private void signalThread() {
            synchronized (io_thread_event) {
                io_thread_event.notify();
            }
        }

        public boolean ioComplete( boolean read, int address, int len, byte[] data) {
            boolean restart = false;
            long system_timestamp = SystemClock.elapsedRealtime();
            if ( address == NAVX_REGISTER_PROC_FIRST ) {
                synchronized(reset_yaw_critical_section) {
                    /* If zero-yaw is requested, discard any received data, to
                       ensure that "stale" yaw data is not delivered.  Once the zero
                       yaw request completes, data will again be delivered. */
                    if ( !request_zero_yaw ) {
                        if (!decodeNavxProcessedData(data,
                                NAVX_REGISTER_PROC_FIRST, data.length)) {
                            setConnected(false);
                        } else {
                            if (curr_sensor_timestamp != last_valid_sensor_timestamp) {
                                addToByteCount(len);
                                incrementUpdateCount();
                                for (int i = 0; i < callbacks.length; i++) {
                                    IDataArrivalSubscriber callback = callbacks[i];
                                    if (callback != null) {
                                        callback.timestampedDataReceived(system_timestamp,
                                                curr_sensor_timestamp,
                                                DeviceDataType.kProcessedData);
                                    }
                                }
                                if (data_type == DeviceDataType.kAll) {
                                    signalThread();
                                    first_bank = false;
                                }
                            } else {
                                duplicate_sensor_data_count++;
                            }

                            if ((curr_sensor_timestamp % 1000) < (last_valid_sensor_timestamp % 1000)) {
                                /* Second roll over.  Start the Hertz accumulator */
                                last_second_hertz = hertz_counter;
                                hertz_counter = 1;
                            } else {
                                hertz_counter++;
                            }

                            last_valid_sensor_timestamp = curr_sensor_timestamp;
                            if (data_type == DeviceDataType.kProcessedData) {
                                if (!cancel_all_reads) {
                                    restart = true;
                                }
                            } else if (data_type == DeviceDataType.kAll) {
                                signalThread();
                            }
                        }
                    }
                }
            } else if ( address == this.NAVX_REGISTER_RAW_FIRST ) {
                if ( !decodeNavxQuatAndRawData(data,
                        NAVX_REGISTER_RAW_FIRST, len) ) {
                    setConnected(false);
                } else {
                    addToByteCount(len);
                    incrementUpdateCount();
                    for ( int i = 0; i < callbacks.length; i++ ) {
                        IDataArrivalSubscriber callback = callbacks[i];
                        if (callback != null) {
                            callback.untimestampedDataReceived(system_timestamp,
                                    DeviceDataType.kQuatAndRawData );
                        }
                    }
                    if ( data_type == DeviceDataType.kQuatAndRawData) {
                        if ( !cancel_all_reads) {
                            restart = true;
                        }
                    } else if ( data_type == DeviceDataType.kAll ) {
                        first_bank = true;
                    }
                }
            }
            return restart;
        }

        @Override
        public void run() {

            final int DIM_MAX_I2C_READ_LEN          = 26;
            final int NAVX_WRITE_COMMAND_BIT        = 0x80;
            DimI2cDeviceReader navxReader[]         = new DimI2cDeviceReader[3];
            I2cDevice navXDevice                    = null;
            DimI2cDeviceWriter navxUpdateRateWriter = null;
            DimI2cDeviceWriter navxZeroYawWriter    = null;

            byte[] update_rate_command  = new byte[1];
            update_rate_command[0]      = (byte)update_rate_hz;
            byte[] zero_yaw_command     = new byte[1];
            zero_yaw_command[0]         = AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_YAW;

            if ( enable_logging ) {
                Log.i("navx_ftc", "Opening device on DIM port " + Integer.toString(dim_port));
            }
            navXDevice = new I2cDevice(dim, dim_port);

            navxReader[0] = new DimI2cDeviceReader(navXDevice, NAVX_I2C_DEV_8BIT_ADDRESS,
                    NAVX_REGISTER_FIRST, DIM_MAX_I2C_READ_LEN);
            navxReader[1] = new DimI2cDeviceReader(navXDevice, NAVX_I2C_DEV_8BIT_ADDRESS,
                    NAVX_REGISTER_PROC_FIRST, DIM_MAX_I2C_READ_LEN);
            navxReader[2] = new DimI2cDeviceReader(navXDevice, NAVX_I2C_DEV_8BIT_ADDRESS,
                    NAVX_REGISTER_RAW_FIRST, DIM_MAX_I2C_READ_LEN);

            /* The board state reader uses synchronous I/O.  The processed and raw data
               readers use an asynchronous IO Completion mechanism.
             */
            navxReader[1].registerIoCallback(this);
            navxReader[2].registerIoCallback(this);

            setConnected(false);

            while ( keep_running ) {
                try {
                    if ( !is_connected ) {
                        this.last_second_hertz = 0;
                        this.last_valid_sensor_timestamp = 0;
                        this.byte_count = 0;
                        this.update_count = 0;
                        this.first_bank = true;
                        this.hertz_counter = 0;
                        this.duplicate_sensor_data_count = 0;
                        byte[] board_data = navxReader[0].startAndWaitForCompletion(I2C_TIMEOUT_MS);
                        if (board_data != null) {
                            if (decodeNavxBoardData(board_data, NAVX_REGISTER_FIRST, board_data.length)) {
                                setConnected(true);
                                first_bank = true;

                                /* To handle the case where the device is reset, reconfigure the */
                                /* update rate whenever reconnecting to the device.               */
                                navxUpdateRateWriter = new DimI2cDeviceWriter(navXDevice,
                                        NAVX_I2C_DEV_8BIT_ADDRESS,
                                        NAVX_WRITE_COMMAND_BIT | IMURegisters.NAVX_REG_UPDATE_RATE_HZ,
                                        update_rate_command);
                                navxUpdateRateWriter.waitForCompletion(I2C_TIMEOUT_MS);

                                board_data = navxReader[0].startAndWaitForCompletion(I2C_TIMEOUT_MS);
                                if ((board_data == null) ||
                                        !decodeNavxBoardData(board_data, NAVX_REGISTER_FIRST, board_data.length)) {
                                    setConnected(false);
                                }
                            }
                        }
                    } else {
                        /* If connected, read sensor data and optionally zero yaw if requested */
                        if (request_zero_yaw) {

                            /* if any reading is underway, wait for it to complete. */
                            cancel_all_reads = true;
                            if (navxReader[1].isBusy()) { navxReader[1].waitForCompletion(I2C_TIMEOUT_MS); }
                            if (navxReader[2].isBusy()) { navxReader[2].waitForCompletion(I2C_TIMEOUT_MS); }
                            cancel_all_reads = false;

                            navxZeroYawWriter = new DimI2cDeviceWriter(navXDevice,
                                    NAVX_I2C_DEV_8BIT_ADDRESS,
                                    NAVX_WRITE_COMMAND_BIT | IMURegisters.NAVX_REG_INTEGRATION_CTL,
                                    zero_yaw_command);
                            navxZeroYawWriter.waitForCompletion(I2C_TIMEOUT_MS);
                            /* After zeroing the yaw, wait one sample time to ensure that */
                            /* a new yaw value (which has been "zeroed") is ready to read. */
                            Thread.sleep(1000 / update_rate_hz);
                            request_zero_yaw = false;
                        }

                        /* Read Processed Data (kProcessedData or kAll) */

                        if ((data_type == DeviceDataType.kProcessedData) ||
                                ((data_type == DeviceDataType.kAll) && first_bank)) {
                            if ( !navxReader[1].isBusy() ) {
                                navxReader[1].start(I2C_TIMEOUT_MS,
                                        (data_type == DeviceDataType.kProcessedData));
                            }
                        }

                        /* Read Quaternion/Raw Data (kQuatAndRawData or kAll) */

                        if ((data_type == DeviceDataType.kQuatAndRawData) ||
                                ((data_type == DeviceDataType.kAll) && !first_bank)) {
                            if ( !navxReader[2].isBusy() ) {
                                navxReader[2].start(I2C_TIMEOUT_MS,
                                        (data_type == DeviceDataType.kQuatAndRawData));
                            }
                        }
                    }
                    /* Wait for a disconnect, read completion or shutdown event */
                    synchronized(io_thread_event) {
                        try {
                            io_thread_event.wait(50);
                        } catch (InterruptedException ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                }
            }

            /* Thread shutdown requested.                                        */
            /* Cancel any pending IO, and wait for them to complete (or timeout) */
            cancel_all_reads = true;

            if (navxReader[1].isBusy()) { navxReader[1].waitForCompletion(I2C_TIMEOUT_MS); }
            if (navxReader[2].isBusy()) { navxReader[2].waitForCompletion(I2C_TIMEOUT_MS); }

            navxReader[1].deregisterIoCallback(this);
            navxReader[2].deregisterIoCallback(this);

            global_dim_state_tracker.reset();

            if ( enable_logging ) {
                Log.i("navx_ftc", "Closing I2C device.");
            }
            navXDevice.close();
        }

        boolean decodeNavxBoardData(byte[] curr_data, int first_address, int len) {
            final int I2C_NAVX_DEVICE_TYPE = 50;
            boolean valid_data;
            if ( curr_data[IMURegisters.NAVX_REG_WHOAMI - first_address] == I2C_NAVX_DEVICE_TYPE ){
                valid_data = true;
                board_id.hw_rev = curr_data[IMURegisters.NAVX_REG_HW_REV - first_address];
                board_id.fw_ver_major = curr_data[IMURegisters.NAVX_REG_FW_VER_MAJOR - first_address];
                board_id.fw_ver_minor = curr_data[IMURegisters.NAVX_REG_FW_VER_MINOR - first_address];
                board_id.type = curr_data[IMURegisters.NAVX_REG_WHOAMI - first_address];

                board_state.gyro_fsr_dps = AHRSProtocol.decodeBinaryUint16(curr_data, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L - first_address);
                board_state.accel_fsr_g = (short) curr_data[IMURegisters.NAVX_REG_ACCEL_FSR_G - first_address];
                board_state.update_rate_hz = curr_data[IMURegisters.NAVX_REG_UPDATE_RATE_HZ - first_address];
                board_state.capability_flags = AHRSProtocol.decodeBinaryUint16(curr_data, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L - first_address);
                board_state.op_status = curr_data[IMURegisters.NAVX_REG_OP_STATUS - first_address];
                board_state.selftest_status = curr_data[IMURegisters.NAVX_REG_SELFTEST_STATUS - first_address];
                board_state.cal_status = curr_data[IMURegisters.NAVX_REG_CAL_STATUS - first_address];
            } else {
                valid_data = false;
            }
            return valid_data;
        }

        boolean doesDataAppearValid( byte[] curr_data ) {
            boolean data_valid = false;
            boolean all_zeros = true;
            boolean all_ones = true;
            for ( int i = 0; i < curr_data.length; i++ ) {
                if ( curr_data[i] != (byte)0 ) {
                    all_zeros = false;
                }
                if ( curr_data[i] != (byte)0xFF) {
                    all_ones = false;
                }
                if ( !all_zeros && !all_ones ) {
                    data_valid = true;
                    break;
                }
            }
            return data_valid;
        }

        boolean decodeNavxProcessedData(byte[] curr_data, int first_address, int len) {
            long timestamp_low, timestamp_high;

            boolean data_valid = doesDataAppearValid(curr_data);
            if ( !data_valid ) {
                Arrays.fill(curr_data, (byte)0);
            }
            curr_sensor_timestamp = (long)AHRSProtocol.decodeBinaryUint32(curr_data, IMURegisters.NAVX_REG_TIMESTAMP_L_L - first_address);
            ahrspos_update.sensor_status = curr_data[IMURegisters.NAVX_REG_SENSOR_STATUS_L - first_address];
            /* Update calibration status from the "shadow" in the upper 8-bits of sensor status. */
            ahrspos_update.cal_status = curr_data[IMURegisters.NAVX_REG_SENSOR_STATUS_H - first_address];
            ahrspos_update.yaw = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_YAW_L - first_address);
            ahrspos_update.pitch = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_PITCH_L - first_address);
            ahrspos_update.roll = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_ROLL_L - first_address);
            ahrspos_update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_HEADING_L - first_address);
            ahrspos_update.fused_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_FUSED_HEADING_L - first_address);
            ahrspos_update.altitude = AHRSProtocol.decodeProtocol1616Float(curr_data, IMURegisters.NAVX_REG_ALTITUDE_I_L - first_address);
            ahrspos_update.linear_accel_x = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_X_L - first_address);
            ahrspos_update.linear_accel_y = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_Y_L - first_address);
            ahrspos_update.linear_accel_z = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_Z_L - first_address);

            return data_valid;
        }

        boolean decodeNavxQuatAndRawData(byte[] curr_data, int first_address, int len) {
            boolean data_valid = doesDataAppearValid(curr_data);
            if ( !data_valid ) {
                Arrays.fill(curr_data, (byte)0);
            }
            ahrspos_update.quat_w   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_W_L-first_address);
            ahrspos_update.quat_x   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_X_L-first_address);
            ahrspos_update.quat_y   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_Y_L-first_address);
            ahrspos_update.quat_z   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_Z_L-first_address);

            ahrspos_update.mpu_temp = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_MPU_TEMP_C_L - first_address);

            raw_data_update.gyro_x  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_X_L-first_address);
            raw_data_update.gyro_y  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_Y_L-first_address);
            raw_data_update.gyro_z  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_Z_L-first_address);
            raw_data_update.accel_x = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_X_L-first_address);
            raw_data_update.accel_y = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_Y_L-first_address);
            raw_data_update.accel_z = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_Z_L-first_address);
            raw_data_update.mag_x   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_X_L-first_address);
            raw_data_update.mag_y   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_Y_L-first_address);
            /* Unfortunately, the 26-byte I2C Transfer limit means we can't transfer the Z-axis magnetometer data.  */
            /* This magnetomer axis typically isn't used, so it's likely not going to be missed.                    */
            //raw_data_update.mag_z   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_Z_L-first_address);

            return data_valid;
        }

    }

    enum DimState {
        UNKNOWN,
        WAIT_FOR_MODE_CONFIG_COMPLETE,
        WAIT_FOR_I2C_TRANSFER_COMPLETION,
        WAIT_FOR_HOST_BUFFER_TRANSFER_COMPLETION,
        READY
    }

    private static DimStateTracker global_dim_state_tracker;
    public DimStateTracker getDimStateTrackerInstance() {
        if ( global_dim_state_tracker == null ) {
            global_dim_state_tracker = new DimStateTracker();
        }
        return global_dim_state_tracker;
    }

    public class DimStateTracker {
        private boolean read_mode;
        private int device_address;
        private int mem_address;
        private int num_bytes;
        private DimState state;

        public DimStateTracker() {
            reset();
        }

        public void reset() {
            read_mode = false;
            device_address = -1;
            mem_address = -1;
            num_bytes = -1;
            state = DimState.UNKNOWN;
        }

        public void setMode( boolean read_mode, int device_address,
                             int mem_address, int num_bytes ) {
            this.read_mode = read_mode;
            this.device_address = device_address;
            this.mem_address = mem_address;
            this.num_bytes = num_bytes;
        }

        public boolean isModeCurrent( boolean read_mode, int device_address,
                                      int mem_address, int num_bytes ) {
            return (( this.read_mode == read_mode ) &&
                    ( this.device_address == device_address ) &&
                    ( this.mem_address == mem_address ) &&
                    ( this.num_bytes == num_bytes ));
        }

        public void setState( DimState new_state ) {
            this.state = new_state;
        }

        public DimState getState() {
            return this.state;
        }
    };

    public class DimI2cDeviceWriter {
        private final I2cDevice device;
        private final int dev_address;
        private final int mem_address;
        private final int num_bytes;
        private boolean done;
        private Object synchronization_event;
        private DimStateTracker state_tracker;

        public DimI2cDeviceWriter(I2cDevice i2cDevice, int i2cAddress, int memAddress, byte[] data) {
            this.device = i2cDevice;
            this.dev_address = i2cAddress;
            this.mem_address = memAddress;
            this.num_bytes = data.length;
            done = false;
            this.synchronization_event = new Object();
            this.state_tracker = getDimStateTrackerInstance();
            i2cDevice.copyBufferIntoWriteBuffer(data);
            i2cDevice.registerForI2cPortReadyCallback(new I2cController.I2cPortReadyCallback() {
                public void portIsReady(int port) {
                    DimI2cDeviceWriter.this.portDone();
                }
            });
            if ( enable_logging ) {
                Log.i("navx_ftc", "Writer registerForPortReadyCallback");
            }
            if ( !state_tracker.isModeCurrent(false,dev_address, mem_address, data.length)) {
                this.state_tracker.setMode(false,dev_address, mem_address, data.length);
                state_tracker.setState(DimState.WAIT_FOR_MODE_CONFIG_COMPLETE);
                device.enableI2cWriteMode(i2cAddress, memAddress, data.length);
                if ( enable_logging ) {
                    Log.i("navx_ftc", "Writer enableI2cWiteMode");
                }

            } else {
                state_tracker.setState(DimState.WAIT_FOR_I2C_TRANSFER_COMPLETION);
                device.setI2cPortActionFlag();
                device.writeI2cCacheToController();
                if ( enable_logging ) {
                    Log.i("navx_ftc", "Writer setI2cPortActionFlag/writeI2cCacheToController");
                }
            }
        }

        public boolean isDone() {
            return this.done;
        }

        private void portDone() {
            DimState dim_state = state_tracker.getState();
            switch ( dim_state ) {

                case WAIT_FOR_MODE_CONFIG_COMPLETE:
                    state_tracker.setState(DimState.WAIT_FOR_I2C_TRANSFER_COMPLETION);
                    device.setI2cPortActionFlag();
                    device.writeI2cCacheToController();
                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Writer WAIT_FOR_MODE_CONFIG_COMPLETE - " +
                                Integer.toString(this.mem_address) + ", " +
                                Integer.toString(this.num_bytes));
                    }
                    break;

                case WAIT_FOR_I2C_TRANSFER_COMPLETION:
                    state_tracker.setState(DimState.READY);
                    device.deregisterForPortReadyCallback();
                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Writer WAIT_FOR_I2C_TRANSFER_COMPLETION; deregisterForPortReadyCallback()");
                    }
                    done = true;
                    synchronized(synchronization_event) {
                        synchronization_event.notify();
                    }
                    break;
            }
        }

        public boolean waitForCompletion( long timeout_ms ) {
            if ( done ) return true;
            boolean success;
            synchronized(synchronization_event) {
                try {
                    synchronization_event.wait(timeout_ms);
                    success = done;
                    if ( !success ) {
                        Log.w("navx_ftc", "Writer waitForCompletion() timeout");
                        /* Unregister the callback.v*/
                        if ( enable_logging ) {
                            Log.i("navx_ftc", "Writer deregisterForPortReadyCallback due to timeout");
                        }
                        device.deregisterForPortReadyCallback();
                    }
                } catch( InterruptedException ex ) {
                    ex.printStackTrace();
                    success = false;
                }
            }
            return success;
        }
    }

    public class DimI2cDeviceReader {
        private final I2cDevice device;
        private final int dev_address;
        private final int mem_address;
        private final int num_bytes;
        private byte[] device_data;
        private Object synchronization_event;
        private boolean registered;
        I2cController.I2cPortReadyCallback callback;
        IoCallback ioCallback;
        DimState dim_state;
        DimStateTracker state_tracker;
        long read_start_timestamp;
        long timeout_ms;
        private boolean busy;
        private boolean continuous_read;
        private boolean continuous_first;

        public DimI2cDeviceReader(I2cDevice i2cDevice, int i2cAddress,
                                  int memAddress, int num_bytes) {
            this.ioCallback = null;
            this.device = i2cDevice;
            this.dev_address = i2cAddress;
            this.mem_address = memAddress;
            this.num_bytes = num_bytes;
            this.synchronization_event = new Object();
            this.registered = false;
            this.state_tracker = getDimStateTrackerInstance();
            this.busy = false;
            this.continuous_read = false;
            this.continuous_first = false;
            this.callback = new I2cController.I2cPortReadyCallback() {
                public void portIsReady(int port) {
                    portDone();
                } };
        }

        public void registerIoCallback( IoCallback ioCallback ) {
            if ( enable_logging ) {
                Log.i("navx_ftc", "Reader Registering reader IO Callbacks.");
            }
            this.ioCallback = ioCallback;
        }

        public void deregisterIoCallback( IoCallback ioCallback ) {
            this.ioCallback = null;
            if ( enable_logging ) {
                Log.i("navx_ftc", "Reader Deregistering reader IO Callbacks.");
            }
        }

        public void start( long timeout_ms, boolean continuous ) {
            start_internal( timeout_ms, continuous, true );
        }

        private void start_internal( long timeout_ms, boolean continuous, boolean first ) {
            device_data = null;
            this.continuous_read = continuous;
            this.continuous_first = continuous && first;
            DimState dim_state = state_tracker.getState();

            /* Start a countdown timer, in case the IO doesn't complete as expected. */
            this.timeout_ms = timeout_ms;
            read_start_timestamp = SystemClock.elapsedRealtime();

            if ( !registered ) {
                if ( enable_logging ) {
                    Log.i("navx_ftc", "Reader registerForI2cPortReadyCallback");
                }
                device.registerForI2cPortReadyCallback(callback);
                registered = true;
            }
            if ( state_tracker.getState() == DimState.UNKNOWN ||
                    (!state_tracker.isModeCurrent(true, dev_address, mem_address, num_bytes ) ) ) {
                /* Force a read-mode transition (if address changed, or of was in write mode) */
                state_tracker.setMode(true, dev_address, mem_address, num_bytes);
                state_tracker.setState(DimState.WAIT_FOR_MODE_CONFIG_COMPLETE);
                device.enableI2cReadMode(dev_address, mem_address, num_bytes);
                if ( enable_logging ) {
                    Log.i("navx_ftc", "Reader enableI2cReadMode");
                }
            } else {
                if ( !device.isI2cPortReady() || !device.isI2cPortInReadMode()) {
                    boolean fail = true;
                }
                /* Already in read mode at the correct address.  Initiate the I2C Bus Read. */
                state_tracker.setState(DimState.WAIT_FOR_I2C_TRANSFER_COMPLETION);
                if ( first || !continuous_read ) {
                    device.setI2cPortActionFlag();
                    device.writeI2cCacheToController();
                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Reader setI2cPortActionFlag/writeI2cCacheToController");
                    }
                } else {
                    /* In this case, the I2C Bus Read was previously initiated in the portDone()
                       callback function, in the WAIT_FOR_I2C_TRANSFER_COMPLETION case.  */
                }
            }
            busy = true;
        }

        public boolean isBusy() {
            long busy_period = SystemClock.elapsedRealtime() - read_start_timestamp;
            if ( busy && ( busy_period >= this.timeout_ms ) ) {
                if ( enable_logging ) {
                    Log.w("navx_ftc", "Reader TIMEOUT!!!");
                }
                busy = false;
                state_tracker.reset();
            }
            return busy;
        }

        private void portDone() {

            DimState dim_state = state_tracker.getState();
            boolean fallthrough = false;
            switch ( dim_state ) {

                case WAIT_FOR_MODE_CONFIG_COMPLETE:
                /* The controller is now in Read Mode with the specified address/len. */
                    device.setI2cPortActionFlag();
                    state_tracker.setState(DimState.WAIT_FOR_I2C_TRANSFER_COMPLETION);
                    device.writeI2cCacheToController();
                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Reader WAIT_FOR_MODE_CONFIG_COMPLETE - " +
                                Integer.toString(this.mem_address) + ", " +
                                Integer.toString(this.num_bytes));
                    }
                    break;

                case WAIT_FOR_I2C_TRANSFER_COMPLETION:
                    state_tracker.setState(DimState.WAIT_FOR_HOST_BUFFER_TRANSFER_COMPLETION);
                    device.readI2cCacheFromController();
                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Reader WAIT_FOR_I2C_TRANSFER_COMPLETION - " +
                                (continuous_read ? "Continuous " : "") +
                                (continuous_first ? "First" : ""));
                    }
                    if ( continuous_read ) {
                    /* Piggy-back the request for the next read.                            */
                    /* For this to work, the write cache already has the read address/len   */
                    /* configured, and the only thing needed to trigger the IO is to change */
                    /* the "port action" flag.  This is a very useful technique, since it   */
                    /* allows another I2C read to start at the same time the read cache is  */
                    /* being transferred from the controller over USB;  Setting only the    */
                    /* port action flag avoid over-writing the other bytes in the cache,    */
                    /* which by may (race condition) have already been updated with the     */
                    /* data read in the just-completed I2C read transaction.                */
                        device.setI2cPortActionFlag();
                        device.writeI2cPortFlagOnlyToController(); /* Write *only* the flag. */
                        if ( continuous_first ) {
                            continuous_first = false;
                            break;
                        } else {
                        /* Fall through to WAIT_FOR_HOST_BUFFER_TRANSFER_COMPLETION case */
                            fallthrough = true;
                        }
                    } else {
                        break;
                    }

                case WAIT_FOR_HOST_BUFFER_TRANSFER_COMPLETION:
                /* The Read Cache has been successfully transferred to this host. */
                    device_data = this.device.getCopyOfReadBuffer();
                    boolean restarted = false;
                    boolean notify = false;

                    if ( enable_logging ) {
                        Log.i("navx_ftc", "Reader WAIT_FOR_HOST_BUFFER_TRANSFER_COMPLETION.  " +
                                (fallthrough ? "(Fallthrough)" : "") +
                                ((device_data != null) ? " Valid Data" : " Null Data"));
                    }

                    if ( this.ioCallback != null ) {
                        boolean repeat = ioCallback.ioComplete(true, this.mem_address,
                                this.num_bytes, device_data);
                        device_data = null;
                        if (repeat) {
                            start_internal(timeout_ms,continuous_read,false);
                            restarted = true;
                        } else {
                            state_tracker.setState(DimState.READY);
                            busy = false;
                            continuous_read = false;
                            notify = true;
                        }
                    } else {
                        state_tracker.setState(DimState.READY);
                        busy = false;
                        continuous_read = false;
                        notify = true;
                    }
                    if ( !restarted ) {
                        if ( enable_logging ) {
                            Log.i("navx_ftc", "Reader deregisterForPortReadyCallback");
                        }
                        device.deregisterForPortReadyCallback();
                        registered = false;
                    }
                /* Finally, after all other processing is complete, notify waiters. */
                    if ( notify ) {
                        synchronized (synchronization_event) {
                            synchronization_event.notify();
                        }
                    }

                    break;
            }
        }

        public byte[] startAndWaitForCompletion(long timeout_ms ) {
            start(timeout_ms, false);
            return waitForCompletion(timeout_ms);
        }

        public byte[] waitForCompletion( long timeout_ms ) {
            if ( !busy && (device_data == null) ) return device_data;
            byte[] data;
            synchronized(synchronization_event) {
                try {
                    synchronization_event.wait(timeout_ms);
                    data = device_data;
                    if ( busy && ( data == null ) ) {
                        Log.w("navx_ftc", "Reader waitForCompletion() timeout");
                        /* Unregister the callback. */
                        if ( enable_logging ) {
                            Log.i("navx_ftc", "Reader deregisterForPortReadyCallback due to timeout");
                        }
                        device.deregisterForPortReadyCallback();
                        registered = false;
                    }
                } catch( InterruptedException ex ) {
                    ex.printStackTrace();
                    data = null;
                }
            }
            return data;
        }

        public byte[] getReadBuffer() {
            return device_data;
        }
    }
}
