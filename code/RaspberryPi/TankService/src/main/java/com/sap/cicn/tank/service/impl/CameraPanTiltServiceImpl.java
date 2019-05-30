/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.service.impl;

import java.math.BigDecimal;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock.ReadLock;
import java.util.concurrent.locks.ReentrantReadWriteLock.WriteLock;

import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.InitializingBean;
import org.springframework.context.annotation.Lazy;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;

import com.sap.cicn.tank.api.domain.CameraAngle;
import com.sap.cicn.tank.api.domain.CameraAngleRange;
import com.sap.cicn.tank.common.exception.BusinessException;
import com.sap.cicn.tank.common.logger.LightLogger;
import com.sap.cicn.tank.hardware.driver.PCA9635Driver;
import com.sap.cicn.tank.service.CameraPanTiltService;

/**
 * @author I311334
 */
@Service
@Lazy(false)
public class CameraPanTiltServiceImpl implements CameraPanTiltService, InitializingBean, DisposableBean {

    private final LightLogger log = LightLogger.getLogger(this);

    private static final byte SERVO_PAN_CHANNEL = 1;
    private static final byte SERVO_TILT_CHANNEL = 0;

    private static final BigDecimal SERVO_CENTER_ANGLE = BigDecimal.valueOf(90);

    private CameraAngle cameraAngle = new CameraAngle();
    private ReentrantReadWriteLock cameraAngleLock = new ReentrantReadWriteLock();
    private ReadLock cameraAngleReadLock = cameraAngleLock.readLock();
    private WriteLock cameraAngleWriteLock = cameraAngleLock.writeLock();

    private Semaphore cameraAngleUpdateSemaphore = new Semaphore(1);
    private Runnable cameraAngleUpdateTask = new Runnable() {
        @Override
        public void run() {
            try {
                while (true) {
                    cameraAngleUpdateSemaphore.acquire();
                    cameraAngleUpdateSemaphore.drainPermits();

                    try {
                        cameraAngleUpdate();
                    } catch (Exception e) {
                        log.error(e, "Error update camera angle, will try again next time. Error message: ", e.getMessage());
                    }
                }
            } catch (InterruptedException e) {
                log.error(e, "Abort update camera angle: thread interrupted.");
            }
        }
    };

    private PCA9635Driver driver;

    /**
     * Construct instance.
     */
    public CameraPanTiltServiceImpl() {
        super();

        cameraAngle.getPanRange().setMin(BigDecimal.valueOf(-90));
        cameraAngle.getPanRange().setMax(BigDecimal.valueOf(90));
        cameraAngle.getPanRange().setCenter(BigDecimal.valueOf(0));
        cameraAngle.getPanRange().setOffset(BigDecimal.valueOf(7));

        cameraAngle.getTiltRange().setMin(BigDecimal.valueOf(-40));
        cameraAngle.getTiltRange().setMax(BigDecimal.valueOf(50));
        cameraAngle.getTiltRange().setCenter(BigDecimal.valueOf(0));
        cameraAngle.getTiltRange().setOffset(BigDecimal.valueOf(-7));

        cameraAngle.setPan(cameraAngle.getPanRange().getCenter());
        cameraAngle.setTilt(cameraAngle.getTiltRange().getCenter());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void afterPropertiesSet() throws Exception {
        driver = PCA9635Driver.getInstance();
        if (driver == null) {
            log.error("Error find PCA9635 device, probably it's local development environment. Camera Pan-Tilt will not start.");
            return;
        }

        driver
                .oscClock(25000000)     // 25MHz
                .pwmFreq(50)            // 50Hz
                .init();

        new Thread(cameraAngleUpdateTask, "CameraAngleUpdateThread").start();

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void destroy() throws Exception {
        driver.close();
    }

    private void cameraAngleUpdate() {
        BigDecimal panAngle = null;
        BigDecimal tiltAngle = null;

        cameraAngleReadLock.lock();
        try {
            panAngle = SERVO_CENTER_ANGLE.add(cameraAngle.getPan()).add(cameraAngle.getPanRange().getOffset());
            tiltAngle = SERVO_CENTER_ANGLE.subtract(cameraAngle.getTilt()).subtract(cameraAngle.getTiltRange().getOffset());
        } finally {
            cameraAngleReadLock.unlock();
        }

        driver.setServoAngle(SERVO_PAN_CHANNEL, panAngle);
        driver.setServoAngle(SERVO_TILT_CHANNEL, tiltAngle);
    }

    @Scheduled(fixedRate = 3 * 1000)
    private void triggerCameraAngleUpdate() {
        cameraAngleUpdateSemaphore.release();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngleRange getCameraAngleRange() {
        cameraAngleReadLock.lock();
        try {
            return new CameraAngleRange(cameraAngle);
        } finally {
            cameraAngleReadLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngleRange setCameraAngleRange(CameraAngleRange cameraAngleRange) {
        if (cameraAngleRange == null) {
            throw new BusinessException("cameraAngleRange is null");
        }

        cameraAngleWriteLock.lock();
        try {
            cameraAngle.copyFrom(cameraAngleRange);

            triggerCameraAngleUpdate();
            return new CameraAngleRange(cameraAngle);
        } finally {
            cameraAngleWriteLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle getCameraAngle() {
        cameraAngleReadLock.lock();
        try {
            return new CameraAngle(cameraAngle);
        } finally {
            cameraAngleReadLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle setCameraAngle(CameraAngle cameraAngle) {
        if (cameraAngle == null) {
            throw new BusinessException("cameraAngle is null");
        }

        cameraAngleWriteLock.lock();
        try {
            this.cameraAngle.setPan(cameraAngle.getPan());
            this.cameraAngle.setTilt(cameraAngle.getTilt());

            triggerCameraAngleUpdate();
            return new CameraAngle(this.cameraAngle);
        } finally {
            cameraAngleWriteLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle centerCamera() {
        return setCameraAngle(new CameraAngle(
                cameraAngle.getPanRange().getCenter(),
                cameraAngle.getTiltRange().getCenter()));
    }

}
