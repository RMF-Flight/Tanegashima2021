# import関連
import csv
import math
import os
import time
import numpy as np
import smbus

# I2C通信のためのインスタンスの生成
i2c = smbus.SMBus(1)

# クラスの定義
class Bmx055():
    # 定数
    GRAVITATIONAL_ACCELERATION  = 9.798     # (m/s^2)
    GEOMAGNETISM = 46.3871                  # (μT)

    # Addresses
    BMX055_ACC_ADDRESS       = 0x19   # Address of BMX055 accelerometer
    BMX055_GYRO_ADDRESS      = 0x69   # Address of BMX055 gyroscope
    BMX055_MAG_ADDRESS       = 0x13   # Address of BMX055 magnetometer

    # Accelerometer registers
    BMX055_ACC_WHOAMI        = 0x00   # should return = 0xFA
    #BMX055_ACC_Reserved     = 0x01
    BMX055_ACC_D_X_LSB       = 0x02
    BMX055_ACC_D_X_MSB       = 0x03
    BMX055_ACC_D_Y_LSB       = 0x04
    BMX055_ACC_D_Y_MSB       = 0x05
    BMX055_ACC_D_Z_LSB       = 0x06
    BMX055_ACC_D_Z_MSB       = 0x07
    BMX055_ACC_D_TEMP        = 0x08
    BMX055_ACC_INT_STATUS_0  = 0x09
    BMX055_ACC_INT_STATUS_1  = 0x0A
    BMX055_ACC_INT_STATUS_2  = 0x0B
    BMX055_ACC_INT_STATUS_3  = 0x0C
    #BMX055_ACC_Reserved     = 0x0D
    BMX055_ACC_FIFO_STATUS   = 0x0E
    BMX055_ACC_PMU_RANGE     = 0x0F
    BMX055_ACC_PMU_BW        = 0x10
    BMX055_ACC_PMU_LPW       = 0x11
    BMX055_ACC_PMU_LOW_POWER = 0x12
    BMX055_ACC_D_HBW         = 0x13
    BMX055_ACC_BGW_SOFTRESET = 0x14
    #BMX055_ACC_Reserved     = 0x15
    BMX055_ACC_INT_EN_0      = 0x16
    BMX055_ACC_INT_EN_1      = 0x17
    BMX055_ACC_INT_EN_2      = 0x18
    BMX055_ACC_INT_MAP_0     = 0x19
    BMX055_ACC_INT_MAP_1     = 0x1A
    BMX055_ACC_INT_MAP_2     = 0x1B
    #BMX055_ACC_Reserved     = 0x1C
    #BMX055_ACC_Reserved     = 0x1D
    BMX055_ACC_INT_SRC       = 0x1E
    #BMX055_ACC_Reserved     = 0x1F
    BMX055_ACC_INT_OUT_CTRL  = 0x20
    BMX055_ACC_INT_RST_LATCH = 0x21
    BMX055_ACC_INT_0         = 0x22
    BMX055_ACC_INT_1         = 0x23
    BMX055_ACC_INT_2         = 0x24
    BMX055_ACC_INT_3         = 0x25
    BMX055_ACC_INT_4         = 0x26
    BMX055_ACC_INT_5         = 0x27
    BMX055_ACC_INT_6         = 0x28
    BMX055_ACC_INT_7         = 0x29
    BMX055_ACC_INT_8         = 0x2A
    BMX055_ACC_INT_9         = 0x2B
    BMX055_ACC_INT_A         = 0x2C
    BMX055_ACC_INT_B         = 0x2D
    BMX055_ACC_INT_C         = 0x2E
    BMX055_ACC_INT_D         = 0x2F
    BMX055_ACC_FIFO_CONFIG_0 = 0x30
    #BMX055_ACC_Reserved     = 0x31
    BMX055_ACC_PMU_SELF_TEST = 0x32
    BMX055_ACC_TRIM_NVM_CTRL = 0x33
    BMX055_ACC_BGW_SPI3_WDT  = 0x34
    #BMX055_ACC_Reserved     = 0x35
    BMX055_ACC_OFC_CTRL      = 0x36
    BMX055_ACC_OFC_SETTING   = 0x37
    BMX055_ACC_OFC_OFFSET_X  = 0x38
    BMX055_ACC_OFC_OFFSET_Y  = 0x39
    BMX055_ACC_OFC_OFFSET_Z  = 0x3A
    BMX055_ACC_TRIM_GPO      = 0x3B
    BMX055_ACC_TRIM_GP1      = 0x3C
    #BMX055_ACC_Reserved     = 0x3D
    BMX055_ACC_FIFO_CONFIG_1 = 0x3E
    BMX055_ACC_FIFO_DATA     = 0x3F

    # BMX055 Gyroscope Registers
    BMX055_GYRO_WHOAMI        = 0x00   # should return = 0x0F
    #BMX055_GYRO_Reserved     = 0x01
    BMX055_GYRO_RATE_X_LSB    = 0x02
    BMX055_GYRO_RATE_X_MSB    = 0x03
    BMX055_GYRO_RATE_Y_LSB    = 0x04
    BMX055_GYRO_RATE_Y_MSB    = 0x05
    BMX055_GYRO_RATE_Z_LSB    = 0x06
    BMX055_GYRO_RATE_Z_MSB    = 0x07
    #BMX055_GYRO_Reserved     = 0x08
    BMX055_GYRO_INT_STATUS_0  = 0x09
    BMX055_GYRO_INT_STATUS_1  = 0x0A
    BMX055_GYRO_INT_STATUS_2  = 0x0B
    BMX055_GYRO_INT_STATUS_3  = 0x0C
    #BMX055_GYRO_Reserved     = 0x0D
    BMX055_GYRO_FIFO_STATUS   = 0x0E
    BMX055_GYRO_RANGE         = 0x0F
    BMX055_GYRO_BW            = 0x10
    BMX055_GYRO_LPM1          = 0x11
    BMX055_GYRO_LPM2          = 0x12
    BMX055_GYRO_RATE_HBW      = 0x13
    BMX055_GYRO_BGW_SOFTRESET = 0x14
    BMX055_GYRO_INT_EN_0      = 0x15
    BMX055_GYRO_INT_EN_1      = 0x16
    BMX055_GYRO_INT_MAP_0     = 0x17
    BMX055_GYRO_INT_MAP_1     = 0x18
    BMX055_GYRO_INT_MAP_2     = 0x19
    BMX055_GYRO_INT_SRC_1     = 0x1A
    BMX055_GYRO_INT_SRC_2     = 0x1B
    BMX055_GYRO_INT_SRC_3     = 0x1C
    #BMX055_GYRO_Reserved     = 0x1D
    BMX055_GYRO_FIFO_EN       = 0x1E
    #BMX055_GYRO_Reserved     = 0x1F
    #BMX055_GYRO_Reserved     = 0x20
    BMX055_GYRO_INT_RST_LATCH = 0x21
    BMX055_GYRO_HIGH_TH_X     = 0x22
    BMX055_GYRO_HIGH_DUR_X    = 0x23
    BMX055_GYRO_HIGH_TH_Y     = 0x24
    BMX055_GYRO_HIGH_DUR_Y    = 0x25
    BMX055_GYRO_HIGH_TH_Z     = 0x26
    BMX055_GYRO_HIGH_DUR_Z    = 0x27
    #BMX055_GYRO_Reserved     = 0x28
    #BMX055_GYRO_Reserved     = 0x29
    #BMX055_GYRO_Reserved     = 0x2A
    BMX055_GYRO_SOC           = 0x31
    BMX055_GYRO_A_FOC         = 0x32
    BMX055_GYRO_TRIM_NVM_CTRL = 0x33
    BMX055_GYRO_BGW_SPI3_WDT  = 0x34
    #BMX055_GYRO_Reserved     = 0x35
    BMX055_GYRO_OFC1          = 0x36
    BMX055_GYRO_OFC2          = 0x37
    BMX055_GYRO_OFC3          = 0x38
    BMX055_GYRO_OFC4          = 0x39
    BMX055_GYRO_TRIM_GP0      = 0x3A
    BMX055_GYRO_TRIM_GP1      = 0x3B
    BMX055_GYRO_BIST          = 0x3C
    BMX055_GYRO_FIFO_CONFIG_0 = 0x3D
    BMX055_GYRO_FIFO_CONFIG_1 = 0x3E

    # BMX055 magnetometer registers
    BMX055_MAG_WHOAMI         = 0x40   # should return = 0x32
    BMX055_MAG_Reserved       = 0x41
    BMX055_MAG_XOUT_LSB       = 0x42
    BMX055_MAG_XOUT_MSB       = 0x43
    BMX055_MAG_YOUT_LSB       = 0x44
    BMX055_MAG_YOUT_MSB       = 0x45
    BMX055_MAG_ZOUT_LSB       = 0x46
    BMX055_MAG_ZOUT_MSB       = 0x47
    BMX055_MAG_ROUT_LSB       = 0x48
    BMX055_MAG_ROUT_MSB       = 0x49
    BMX055_MAG_INT_STATUS     = 0x4A
    BMX055_MAG_PWR_CNTL1      = 0x4B
    BMX055_MAG_PWR_CNTL2      = 0x4C
    BMX055_MAG_INT_EN_1       = 0x4D
    BMX055_MAG_INT_EN_2       = 0x4E
    BMX055_MAG_LOW_THS        = 0x4F
    BMX055_MAG_HIGH_THS       = 0x50
    BMX055_MAG_REP_XY         = 0x51
    BMX055_MAG_REP_Z          = 0x52

    # Trim Extended Registers
    BMM050_DIG_X1             = 0x5D   # needed for magnetic field calculation
    BMM050_DIG_Y1             = 0x5E
    BMM050_DIG_Z4_LSB         = 0x62
    BMM050_DIG_Z4_MSB         = 0x63
    BMM050_DIG_X2             = 0x64
    BMM050_DIG_Y2             = 0x65
    BMM050_DIG_Z2_LSB         = 0x68
    BMM050_DIG_Z2_MSB         = 0x69
    BMM050_DIG_Z1_LSB         = 0x6A
    BMM050_DIG_Z1_MSB         = 0x6B
    BMM050_DIG_XYZ1_LSB       = 0x6C
    BMM050_DIG_XYZ1_MSB       = 0x6D
    BMM050_DIG_Z3_LSB         = 0x6E
    BMM050_DIG_Z3_MSB         = 0x6F
    BMM050_DIG_XY2            = 0x70
    BMM050_DIG_XY1            = 0x71


    ### Set initial input parameters　###

    # Accelerometer Full Scale options (Ascale)
    AFS_2G  = 0x03
    AFS_4G  = 0x05
    AFS_8G  = 0x08
    AFS_16G = 0x0C

    # Accelerometer BandWidths  (ACCBW, ABW)
    ABW_8Hz   = 0   # 7.81  Hz,  64 ms update time
    ABW_16Hz  = 1   # 15.63 Hz,  32 ms update time
    ABW_31Hz  = 2   # 31.25 Hz,  16 ms update time
    ABW_63Hz  = 3   # 62.5  Hz,   8 ms update time
    ABW_125Hz = 4   # 125   Hz,   4 ms update time
    ABW_250Hz = 5   # 250   Hz,   2 ms update time
    ABW_500Hz = 6   # 500   Hz,   1 ms update time
    ABW_100Hz = 7   # 1000  Hz, 0.5 ms update time

    # Gyroscope Full Scale option  (Gscale)
    GFS_2000DPS = 0
    GFS_1000DPS = 1
    GFS_500DPS  = 2
    GFS_250DPS  = 3
    GFS_125DPS  = 4

    # Gyroscope Output Data Rate and filter BandWidth  (GODRBW)
    G_2000Hz523Hz = 0   # 2000 Hz ODR and unfiltered (bandwidth 523Hz)
    G_2000Hz230Hz = 1
    G_1000Hz116Hz = 2
    G_400Hz47Hz   = 3
    G_200Hz23Hz   = 4
    G_100Hz12Hz   = 5
    G_200Hz64Hz   = 6
    G_100Hz32Hz   = 7   # 100 Hz ODR and 32 Hz bandwidth

    # Magnetmeter Output Data Rate  (MODR)
    MODR_10Hz = 0   # 10 Hz ODR
    MODR_2Hz  = 1   #  2 Hz ODR
    MODR_6Hz  = 2   #  6 Hz ODR
    MODR_8Hz  = 3   #  8 Hz ODR
    MODR_15Hz = 4   # 15 Hz ODR
    MODR_20Hz = 5   # 20 Hz ODR
    MODR_25Hz = 6   # 25 Hz ODR
    MODR_30Hz = 7   # 30 Hz ODR

    # Magnetmeter active mode   (Mmode)
    lowPower         = 0   # rms noise ~1.0 microTesla, 0.17 mA power
    Regular          = 1   # rms noise ~0.6 microTesla, 0.5  mA power
    enhancedRegular  = 2   # rms noise ~0.5 microTesla, 0.8  mA power
    highAccuracy     = 3   # rms noise ~0.3 microTesla, 4.9  mA power

    # コンストラクタ
    def __init__(self, Ascale=AFS_2G, ACCBW=ABW_16Hz, Gscale=GFS_250DPS, GODRBW=G_200Hz23Hz, Mmode=Regular, MODR=MODR_10Hz, path_BMX055_calibration_data='./BMX055_calibration_data/'):
        # Configure Accelerometer
        i2c.write_byte_data(self.BMX055_ACC_ADDRESS, self.BMX055_ACC_PMU_RANGE, Ascale & 0x0F)        # Set accelerometer full scale
        i2c.write_byte_data(self.BMX055_ACC_ADDRESS, self.BMX055_ACC_PMU_BW, (0x08 | ACCBW) & 0x0F)   # Set accelerometer bandwidth
        i2c.write_byte_data(self.BMX055_ACC_ADDRESS, self.BMX055_ACC_D_HBW, 0x00)                     # Use filtered data

        # Configure Gyroscope
        i2c.write_byte_data(self.BMX055_GYRO_ADDRESS, self.BMX055_GYRO_RANGE, Gscale)   # set Gyroscope full scale
        i2c.write_byte_data(self.BMX055_GYRO_ADDRESS, self.BMX055_GYRO_BW, GODRBW)      # set GYRO ODR and Bandwidth

        # Configure Magnetometer
        while True:
            try:
                i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_PWR_CNTL1, 0x82)   # Softreset magnetometer, ends up in sleep mode
                break
            except OSError:
                time.sleep(0.1)
        time.sleep(0.1)
        i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_PWR_CNTL1, 0x01)        # Wake up magnetometer
        time.sleep(0.1)
        i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_PWR_CNTL2, MODR << 3)   # Normal mode

        # Set up four standard configurations for the magnetometer
        if Mmode == self.lowPower:
            # Low-power
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_XY, 0x01)   # 3 repetitions (oversampling)
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_Z,  0x02)   # 3 repetitions (oversampling)
        elif Mmode == self.Regular:
            # Regular
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_XY, 0x04)   #  9 repetitions (oversampling)
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_Z,  0x16)   # 15 repetitions (oversampling)
        elif Mmode == self.enhancedRegular:
            # Enhanced Regular
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_XY, 0x07)   # 15 repetitions (oversampling)
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_Z,  0x22)   # 27 repetitions (oversampling)
        elif Mmode == self.highAccuracy:
            # High Accuracy
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_XY, 0x17)   # 47 repetitions (oversampling)
            i2c.write_byte_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_REP_Z,  0x51)   # 83 repetitions (oversampling)


        ### get sensor resolutions, only need to do this once ###

        # Possible accelerometer scales (and their register bit settings) are:
        # 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
        # BMX055 ACC data is signed 12 bit
        if Ascale == self.AFS_2G:
            self.aRes = 2.0/2048.0
        elif Ascale == self.AFS_4G:
            self.aRes = 4.0/2048.0
        elif Ascale == self.AFS_8G:
            self.aRes = 8.0/2048.0
        elif Ascale == self.AFS_16G:
            self.aRes = 16.0/2048.0
        
        # Possible gyro scales (and their register bit settings) are:
        # 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
        # BMX055 GYRO data is signed 16 bit
        if Gscale == self.GFS_125DPS:
            self.gRes = 124.87/32768.0   # per data sheet, not exactly 125!?
        elif Gscale == self.GFS_250DPS:
            self.gRes = 249.75/32768.0
        elif Gscale == self.GFS_500DPS:
            self.gRes = 499.5/32768.0
        elif Gscale == self.GFS_1000DPS:
            self.gRes = 999.0/32768.0
        elif Gscale == self.GFS_2000DPS:
            self.gRes = 1998.0/32768.0
        
        # magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
        self.mRes = 1/1.6

        # read the magnetometer calibration data
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_X1, 1)
        self.dig_x1 = self._u2s(raw_data[0], 8)
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_X2, 1)
        self.dig_x2 = self._u2s(raw_data[0], 8)
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_Y1, 1)
        self.dig_y1 = self._u2s(raw_data[0], 8)
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_Y2, 1)
        self.dig_y2 = self._u2s(raw_data[0], 8)
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_XY1, 1)
        self.dig_xy1 = raw_data[0]
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMM050_DIG_XY2, 1)
        self.dig_xy2 = self._u2s(raw_data[0], 8)
        raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMM050_DIG_Z1_LSB, 2)
        self.dig_z1  = (raw_data[1] << 8) | raw_data[0]
        raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMM050_DIG_Z2_LSB, 2)
        self.dig_z2 = self._u2s((raw_data[1] << 8) | raw_data[0], 16)
        raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMM050_DIG_Z3_LSB, 2)
        self.dig_z3 = self._u2s((raw_data[1] << 8) | raw_data[0], 16)
        raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMM050_DIG_Z4_LSB, 2)
        self.dig_z4 = self._u2s((raw_data[1] << 8) | raw_data[0], 16)
        raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMM050_DIG_XYZ1_LSB, 2)
        self.dig_xyz1 = (raw_data[1] << 8) | raw_data[0]

        #fastcompaccelBMX055(accelBias)     # 未実装
        #magcalBMX055(magBias)              # 未実装

        # Set euler angles
        self.e = np.array([0, 0, 0])

        # Set quaternion
        self.q = np.array([1, 0, 0, 0])
        
        # Set paths
        self.path_BMX055_calibration_data = path_BMX055_calibration_data
        self.path_acc_calibration_log = self.path_BMX055_calibration_data + 'acc_calibration_log.csv'
        self.path_acc_correction_value = self.path_BMX055_calibration_data + 'acc_correction_value.csv'
        self.path_gyro_calibration_log = self.path_BMX055_calibration_data + 'gyro_calibration_log.csv'
        self.path_gyro_correction_value = self.path_BMX055_calibration_data + 'gyro_correction_value.csv'
        self.path_mag_calibration_log = self.path_BMX055_calibration_data + 'mag_calibration_log.csv'
        self.path_mag_correction_value = self.path_BMX055_calibration_data + 'mag_correction_value.csv'

        # Make directory for storing data
        os.makedirs(self.path_BMX055_calibration_data, exist_ok=True)

    # Convert an unsigned number to a signed number.
    def _u2s(self, number, bit):
        if number < 2**(bit-1):
            return number
        else:
            return number - 2**bit
    
    # Get acceleration value
    def get_acc_value(self):
        # Read the six raw data registers into data array
        raw_data = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMX055_ACC_D_X_LSB, 6)
        if (raw_data[0] & 0x01) and (raw_data[2] & 0x01) and (raw_data[4] & 0x01):
            # Check that all 3 axes have new data
            # Turn the MSB and LSB into a signed 12-bit value
            x = self._u2s(((raw_data[1] << 8) | raw_data[0]) >> 4, 12)
            y = self._u2s(((raw_data[3] << 8) | raw_data[2]) >> 4, 12)
            z = self._u2s(((raw_data[5] << 8) | raw_data[4]) >> 4, 12)

            acc = np.array([x, y, z])
            acc = acc * self.aRes * self.GRAVITATIONAL_ACCELERATION   # (m/s^2)
            if os.path.exists(self.path_acc_correction_value ):
                acc_correction_value = np.loadtxt(self.path_acc_correction_value , delimiter=',')
                offset, gain = acc_correction_value
                acc = self.GRAVITATIONAL_ACCELERATION * (acc - offset) / gain
            return acc

    # Get angular velocity value
    def get_gyro_value(self):
        # Read the six raw data registers sequentially into data array
        raw_data = i2c.read_i2c_block_data(self.BMX055_GYRO_ADDRESS, self.BMX055_GYRO_RATE_X_LSB, 6)
        # Turn the MSB and LSB into a signed 16-bit value
        x = self._u2s((raw_data[1] << 8) | raw_data[0], 16)
        y = self._u2s((raw_data[3] << 8) | raw_data[2], 16)
        z = self._u2s((raw_data[5] << 8) | raw_data[4], 16)

        gyro = np.array([x, y, z])
        gyro = gyro * self.gRes * math.pi / 180  # (rad/s)
        if os.path.exists(self.path_gyro_correction_value):
            gyro_correction_value = np.loadtxt(self.path_gyro_correction_value, delimiter=',')
            offset = gyro_correction_value
            gyro = gyro - offset
        return gyro

    # Get geomagnetism data
    def get_mag_value(self):
        # Read the eight raw data registers sequentially into data array
        error_array = [1, 0, 1, 0, 1, 0, 0, 0]
        while True:
            raw_data = i2c.read_i2c_block_data(self.BMX055_MAG_ADDRESS, self.BMX055_MAG_XOUT_LSB, 8)
            if raw_data != error_array:
                break
            time.sleep(0.1)
        if (raw_data[6] & 0x01):   # Check if data ready status bit is set
            mdata_x = self._u2s(((raw_data[1] << 8) | raw_data[0]) >> 3, 13)   # 13-bit signed integer for x-axis field
            mdata_y = self._u2s(((raw_data[3] << 8) | raw_data[2]) >> 3, 13)   # 13-bit signed integer for y-axis field
            mdata_z = self._u2s(((raw_data[5] << 8) | raw_data[4]) >> 1, 15)   # 15-bit signed integer for z-axis field
            data_r  = ((raw_data[7] << 8) | raw_data[6]) >> 2            # 14-bit unsigned integer for Hall resistance
 
            # calculate temperature compensated 16-bit magnetic fields
            tt = data_r if data_r != 0 else self.dig_xyz1
            temp = self._u2s((((self.dig_xyz1 << 14)/tt) - 0x4000), 16)
            x = ((mdata_x * (((((self.dig_xy2 * (int(temp ** 2) >> 7)) + (int(temp) * (self.dig_xy1 << 7))) >> 9) + 0x100000) * (self.dig_x2 + 0xA0)) >> 12) >> 13) + (self.dig_x1 << 3)
            y = ((mdata_y * (((((self.dig_xy2 * (int(temp ** 2) >> 7)) + (int(temp) * (self.dig_xy1 << 7))) >> 9) + 0x100000) * (self.dig_y2 + 0xA0)) >> 12) >> 13) + (self.dig_y1 << 3)
            z = (((mdata_z - self.dig_z4) << 15) - ((self.dig_z3 * (data_r - self.dig_xyz1))>>2))/(self.dig_z2 + (((self.dig_z1 * (data_r << 1))+(1<<15))>>16))

            mag = np.array([-y, x, z])    # Adjust the axis
            mag = mag * self.mRes * 0.1   # (μT)
            if os.path.exists(self.path_mag_correction_value):
                mag_correction_value = np.loadtxt(self.path_mag_correction_value, delimiter=',')
                offset, gain, _ = mag_correction_value
                mag = self.GEOMAGNETISM * (mag - offset) / gain
            return mag

    # Get atmospheric temperature
    def get_temperature(self):
        c = i2c.read_i2c_block_data(self.BMX055_ACC_ADDRESS, self.BMX055_ACC_D_TEMP, 1)   # Read the raw data register
        temp_count = self._u2s(c[0] << 8, 16) >> 8   # Turn the byte into a signed 8-bit integer
        temperature = (temp_count * 0.5) + 23.0     # (°C)
        return temperature
    
    # Calibrate the accelerometer
    def calibrate_accelerometer(self, count=50):
        # 以前のデータがある場合、本当に較正を行うか尋ねます
        if os.path.exists(self.path_acc_calibration_log):
            print('There are previous data.')
            print('Do you really want to start the calibration? [Y/n]', end=' ')
            if input() != 'Y':
                return

        # 以前の関連ファイルを削除します
        if os.path.exists(self.path_acc_calibration_log):
            os.remove(self.path_acc_calibration_log)
        if os.path.exists(self.path_acc_correction_value):
            os.remove(self.path_acc_correction_value)
        
        # キャリブレーション実行部
        # カウントダウンが終わるまで表示された軸方向を上に向きにしたままにしてください
        preparation_time = 7
        direction = ('+x', '-x', '+y', '-y', '+z', '-z')
        arr = np.zeros(6).reshape((2, 3))
        for i in range(6):
            print('Upward ', direction[i])
            for t in range(preparation_time, 0, -1):
                print('It will start in', t)
                time.sleep(1)
            d, m = divmod(i, 2)
            tmp = 0
            for c in range(count):
                acc = self.get_acc_value()
                with open(self.path_acc_calibration_log, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow(acc)
                print(acc, count-c)
                tmp += acc[d]
                time.sleep(0.1)
            arr[m][d] = tmp / count
        offset = (arr[0] + arr[1]) / 2
        gain = (arr[0] - arr[1]) / 2

        # ファイルの一行目にオフセットデータ、二行目にスケールを保存します
        with open(self.path_acc_correction_value, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(offset)
            writer.writerow(gain)
        print('offset ', offset)
        print('gain   ', gain)

    # Calibrate the gyroscope
    def calibrate_gyroscope(self, count=100):
        # 以前のデータがある場合、本当に較正を行うか尋ねます
        if os.path.exists(self.path_gyro_correction_value):
            print('There are previous data.')
            print('Do you really want to start the calibration? [Y/n]', end=' ')
            if input() != 'Y':
                return

        # 以前の関連ファイルを削除します
        if os.path.exists(self.path_gyro_calibration_log):
            os.remove(self.path_gyro_calibration_log)
        if os.path.exists(self.path_gyro_correction_value):
            os.remove(self.path_gyro_correction_value)

        # キャリブレーション実行部
        # カウントダウンが終わるまで触らないでください
        arr = np.zeros(3)
        for c in range(count):
            gyro = self.get_gyro_value()
            with open(self.path_gyro_calibration_log, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(gyro)
            print(gyro, count-c)
            arr += gyro
            time.sleep(0.1)
        offset = arr / count

        # ファイルの一行目にオフセットデータ、二行目にスケールを保存します
        with open(self.path_gyro_correction_value, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(offset)
        print('offset ', offset)

    # Calibrate the geomagnetometer
    def calibrate_geomagnetometer(self, count=300):
        # 以前のデータがある場合、本当に較正を行うか尋ねます
        if os.path.exists(self.path_mag_correction_value):
            print('There are previous data.')
            print('Do you really want to start the calibration? [Y/n]', end=' ')
            if input() != 'Y':
                return

        # 以前の関連ファイルを削除します
        if os.path.exists(self.path_mag_calibration_log):
            os.remove(self.path_mag_calibration_log)
        if os.path.exists(self.path_mag_correction_value):
            os.remove(self.path_mag_correction_value)
        
        # キャリブレーション実行部
        # カウントダウンが終わるまで八の字に振り続けてください
        # 体の向きを変えたり、持ち手を変えたりするとよいデータが得られやすいです
        arr = np.zeros((6, 7))
        preparation_time = 5
        for t in range(preparation_time, 0, -1):
            print('It will start in', t)
            time.sleep(1)
        for c in range(count):
            mag = self.get_mag_value()
            with open(self.path_mag_calibration_log, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(mag)
            print(mag, count-c)
            temp1 = np.append(mag**2, mag)
            temp2 = np.append(temp1, 1)
            arr += np.dot(temp1.reshape((6, 1)), temp2.reshape((1, 7)))
            time.sleep(0.1)
        arr1 = np.linalg.inv(np.delete(arr, 6, 1))
        arr2 = arr[:, 6].reshape((6, 1))
        coef = np.ravel(np.dot(arr1, arr2))
        offset = - (coef[3:] / coef[:3]) / 2
        temp = 1 - np.sum(coef[3:] * coef[:3]) / 2
        gain = np.sqrt(temp / coef[:3])
        geo = np.full(3, self.GEOMAGNETISM)

        # ファイルの一行目にオフセットデータ、二行目にスケール、三行目に国土地理院から得た地磁気の値を保存します
        with open(self.path_mag_correction_value, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(offset)
            writer.writerow(gain)
            writer.writerow(geo)
        print('offset ', offset)
        print('gain    ', gain)
    
    # Calibrate all sensor
    def calibrate(self):
        print('calibrate the accelerometer')
        self.calibrate_accelerometer()
        print('calibrate the gyroscope')
        self.calibrate_gyroscope()
        print('calibrate the geomagnetometer')
        self.calibrate_geomagnetometer()

    # Calculate the Eulerian angle from acceleration and geomagnetism
    def get_euler_angle_using_acc_mag_value(self):
        acc = self.get_acc_value()
        mag = self.get_mag_value()
        roll = math.atan2(acc[1], acc[2])
        pitch = math.atan2(-acc[0], math.sqrt(acc[1]**2+acc[2]**2))
        sr, sp = math.sin(roll), math.sin(pitch)
        cr, cp = math.cos(roll), math.cos(pitch)
        temp = np.dot(np.array([[cp, sp*sr, sp*cr], [0, cr, -sr]]), mag.reshape((3, 1)))
        yaw = math.atan2(-temp[1], temp[0])
        euler_angle = np.array([roll, pitch, yaw])
        return euler_angle

    # Updates the Eulerian angle using the angular velocity
    def get_euler_angle_using_gyro_value(self, last_euler_angle, deltat):
        gyro = self.get_gyro_value()
        d_rot = gyro.reshape((3, 1)) * deltat
        sr, sp, _ = np.sin(last_euler_angle)
        cr, cp, _ = np.cos(last_euler_angle)
        conv = np.array([[1, sr*sp/cp, cr*sp/cp], [0, cr, -sr], [0, sr/cp, cr/cp]])
        d_euler_angle = np.ravel(np.dot(conv, d_rot))
        euler_angle = last_euler_angle + d_euler_angle
        return euler_angle

    # Update the euler angle by complementing
    def update_euler_angle(self, deltat, high=0.9):
        eg = self.get_euler_angle_using_gyro_value(self.e, deltat)*high
        eam = self.get_euler_angle_using_acc_mag_value()*(1-high)
        self.e = eg + eam

    # Update the quaternion with Madgwick filter
    def update_quaternion_with_Madgwick_filter(self, deltat):
        acc = self.get_acc_value()
        gyro = self.get_gyro_value()
        mag = self.get_mag_value()

        GyroMeasError = np.pi * (40/180)
        beta = (3/4)**(1/2) * GyroMeasError

        # Auxiliary variables to avoid repeated arithmetic
        q1, q2, q3, q4 = self.q
        q = self.q.reshape((4, 1))
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement magnetometer measurement
        acc /= np.linalg.norm(acc)
        mag /= np.linalg.norm(mag)
        ax, ay, az = acc
        mx, my, mz = mag
 
        # Reference direction of Earth's magnetic field
        hx = mx*(q1q1 + q2q2 - q3q3 - q4q4) + 2*my*(q2q3 - q1q4) + 2*mz*(q2q4 + q1q3) 
        hy = my*(q1q1 - q2q2 + q3q3 - q4q4) + 2*mx*(q2q3 + q1q4) + 2*mz*(q3q4 - q1q2)
        _2bx = (hx*hx + hy*hy)**(1/2)
        _2bz = mz*(q1q1 - q2q2 - q3q3 + q4q4) + 2*mx*(q2q4 - q1q3) + 2*my*(q3q4 - q1q2)
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient decent algorithm corrective step
        fg = np.array([[2*(q2q4 - q1q3) - ax], [2*(q1q2+q3q4) - ay], [2*(0.5 - q2q2 - q3q3) - az]])
        jg = np.array([[-q3, q2, 0], [q4, q1, -2*q2], [-q1, q4, -2*q3], [q2, q3, 0]]) * 2
        fb = np.array([[_2bx*(0.5-q3q3-q4q4) + _2bz*(q2q4-q1q3) - mx], [_2bx*(q2q3-q1q4) + _2bz*(q1q2+q3q4) - my], [_2bx*(q1q3+q2q4)+_2bz*(0.5-q2q2-q3q3) - mz]])
        jb = np.array([[-_2bz*q3, -_2bx*q4+_2bz*q2, _2bx*q3], [_2bz*q4, _2bx*q3+_2bz*q1, _2bx*q4-_4bz*q2], [-_4bx*q3-_2bz*q1, _2bx*q2+_2bz*q4, _2bx*q1-_4bz*q3], [-_4bx*q4+_2bz*q2, -_2bx*q1+_2bz*q3, _2bx*q2]])
        s = np.dot(jg, fg) + np.dot(jb, fb)
        n = np.linalg.norm(s)
        s = s / n
 
        # Compute rate of change of quaternion
        qs = np.array([[-q2, -q3, -q4], [q1, -q4, q3], [q4, q1, -q2], [-q3, q2, q1]])
        gy = np.resize(gyro, (3, 1))
        qDot = 0.5*np.dot(qs, gy) - beta*s
 
        # Integrate to yield quaternion
        q = q + qDot*deltat
        n = np.linalg.norm(q)
        self.q = np.ravel(q / n)    

    # Convert quaternion to Eulerian angle
    def qua2eul(self, q):
        q1, q2, q3, q4 = q
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        roll = math.atan2(2*(q1q2 + q3q4), q1q1 - q2q2 - q3q3 + q4q4)
        pitch = math.asin(2*(q1q3 - q2q4))
        yaw = math.atan2(2*(q1q4 + q2q3), q1q1 + q2q2 - q3q3 - q4q4)

        euler_angle = np.array([roll, pitch, yaw])
        return euler_angle


if __name__ == "__main__":
    bmx = Bmx055()
    np.set_printoptions(precision=6, suppress=True)
    deltat = 0.1
    while True:
        bmx.update_quaternion_with_Madgwick_filter(deltat)
        print(np.rad2deg(bmx.qua2eul(bmx.q)))
        time.sleep(deltat)