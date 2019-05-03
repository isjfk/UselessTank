/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.service.impl;

import java.math.BigDecimal;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicReference;

import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.InitializingBean;
import org.springframework.context.annotation.Lazy;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;

import com.sap.cicn.tank.api.domain.CameraAngle;
import com.sap.cicn.tank.api.domain.CameraAngleRange;
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

    private AtomicReference<CameraAngle> cameraAngleRef = new AtomicReference<>();
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

        CameraAngleRange cameraAngleRange = new CameraAngleRange();

        cameraAngleRange.setPanMin(BigDecimal.valueOf(-50));
        cameraAngleRange.setPanCenter(BigDecimal.valueOf(0));
        cameraAngleRange.setPanMax(BigDecimal.valueOf(50));
        cameraAngleRange.setPanOffset(BigDecimal.valueOf(7));

        cameraAngleRange.setTiltMin(BigDecimal.valueOf(-50));
        cameraAngleRange.setTiltCenter(BigDecimal.valueOf(0));
        cameraAngleRange.setTiltMax(BigDecimal.valueOf(50));
        cameraAngleRange.setTiltOffset(BigDecimal.valueOf(7));

        CameraAngle cameraAngle = new CameraAngle();

        cameraAngle.setRange(cameraAngleRange);
        cameraAngle.setPan(cameraAngleRange.getPanCenter());
        cameraAngle.setTilt(cameraAngleRange.getTiltCenter());

        cameraAngleRef.set(cameraAngle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void afterPropertiesSet() throws Exception {
        driver = PCA9635Driver.getInstance()
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
        CameraAngle angle = cameraAngleRef.get();
        CameraAngleRange angleRange = angle.getRange();
        driver.setServoAngle(SERVO_PAN_CHANNEL, SERVO_CENTER_ANGLE.add(angle.getPan()).add(angleRange.getPanOffset()));
        driver.setServoAngle(SERVO_TILT_CHANNEL, SERVO_CENTER_ANGLE.add(angle.getTilt()).add(angleRange.getTiltOffset()));
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
        return cameraAngleRef.get().getRange().clone();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setCameraAngleRange(CameraAngleRange cameraAngleRange) {
        CameraAngleRange newAngleRange = cameraAngleRange.clone();

        CameraAngle orgAngle = cameraAngleRef.get();
        CameraAngle newAngle = new CameraAngle(orgAngle.getPan(), orgAngle.getTilt(), newAngleRange);
        while (!this.cameraAngleRef.compareAndSet(orgAngle, newAngle)) {
            orgAngle = cameraAngleRef.get();
            newAngle = new CameraAngle(orgAngle.getPan(), orgAngle.getTilt(), newAngleRange);
        }

        triggerCameraAngleUpdate();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle getCameraAngle() {
        return cameraAngleRef.get().clone();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle setCameraAngle(CameraAngle angle) {
        CameraAngle orgAngle = cameraAngleRef.get();
        CameraAngle newAngle = new CameraAngle(angle.getPan(), angle.getTilt(), orgAngle.getRange());
        while (!this.cameraAngleRef.compareAndSet(orgAngle, newAngle)) {
            orgAngle = cameraAngleRef.get();
            newAngle = new CameraAngle(angle.getPan(), angle.getTilt(), orgAngle.getRange());
        }

        triggerCameraAngleUpdate();
        return newAngle.clone();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle centerCamera() {
        CameraAngleRange range = getCameraAngleRange();
        return setCameraAngle(new CameraAngle(range.getPanCenter(), range.getTiltCenter()));
    }

}
