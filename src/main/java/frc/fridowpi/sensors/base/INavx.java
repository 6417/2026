package frc.fridowpi.sensors.base;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public interface INavx {
    void init();

    float getPitch();

    float getRoll();

    float getYaw();

    float getCompassHeading();

    void zeroYaw();

    boolean isCalibrating();

    boolean isConnected();

    double getByteCount();

    int getActualUpdateRate();

    int getRequestedUpdateRate();

    double getUpdateCount();

    long getLastSensorTimestamp();

    float getWorldLinearAccelX();

    float getWorldLinearAccelY();

    float getWorldLinearAccelZ();

    boolean isMoving();

    boolean isRotating();

    float getBarometricPressure();

    float getAltitude();

    boolean isAltitudeValid();

    float getFusedHeading();

    boolean isMagneticDisturbance();

    boolean isMagnetometerCalibrated();

    float getQuaternionW();

    float getQuaternionX();

    float getQuaternionY();

    float getQuaternionZ();

    void resetDisplacement();

    float getVelocityX();

    float getVelocityY();

    float getVelocityZ();

    float getDisplacementX();

    float getDisplacementY();

    float getDisplacementZ();

    /* NOTE! ITimestampedDataSubscriber does not exist on 2026 studica */
    //boolean registerCallback(ITimestampedDataSubscriber callback, Object callback_context);

    //boolean deregisterCallback(ITimestampedDataSubscriber callback);

    double getAngle();

    double getRate();

    void setAngleAdjustment(double adjustment);

    double getAngleAdjustment();

    void reset();

    Rotation2d getRotation2d();

    float getRawGyroX();

    float getRawGyroY();

    float getRawGyroZ();

    float getRawAccelX();

    float getRawAccelY();

    float getRawAccelZ();

    float getRawMagX();

    float getRawMagY();

    float getRawMagZ();

    float getPressure();

    float getTempC();

    AHRS.BoardYawAxis getBoardYawAxis();

    String getFirmwareVersion();

    void enableLogging(boolean enable);

    void enableBoardlevelYawReset(boolean enable);

    boolean isBoardlevelYawResetEnabled();

    short getGyroFullScaleRangeDPS();

    short getAccelFullScaleRangeG();
}
