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
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.sap.cicn.tank.api.domain.TankControl;
import com.sap.cicn.tank.api.domain.TankControlRange;
import com.sap.cicn.tank.common.logger.LightLogger;
import com.sap.cicn.tank.service.PortService;
import com.sap.cicn.tank.service.TankControlService;

/**
 * @author I311334
 */
@Service
public class TankControlServiceImpl implements TankControlService, InitializingBean, DisposableBean {

    private final LightLogger log = LightLogger.getLogger(this);

    private TankControl tankControl = new TankControl();
    private ReentrantReadWriteLock tankControlLock = new ReentrantReadWriteLock();
    private ReadLock tankControlReadLock = tankControlLock.readLock();
    private WriteLock tankControlWriteLock = tankControlLock.writeLock();

    private Semaphore tankControlUpdateSemaphore = new Semaphore(0);
    private Runnable tankControlUpdateTask = new Runnable() {
        @Override
        public void run() {
            try {
                while (true) {
                    tankControlUpdateSemaphore.acquire();
                    tankControlUpdateSemaphore.drainPermits();

                    try {
                        tankControlUpdate();
                    } catch (Exception e) {
                        log.error(e, "Error update tank control, will try again next time. Error message: ", e.getMessage());
                    }
                }
            } catch (InterruptedException e) {
                log.error(e, "Abort update tank control: thread interrupted.");
            }
        }
    };

    @Autowired
    private PortService portService;

    /**
     * Construct instance.
     *
     */
    public TankControlServiceImpl() {
        super();

        tankControl.getThrottleRange().setMin(BigDecimal.valueOf(-100));
        tankControl.getThrottleRange().setMax(BigDecimal.valueOf(100));
        tankControl.getThrottleRange().setCenter(BigDecimal.valueOf(0));
        tankControl.getThrottleRange().setOffset(BigDecimal.valueOf(0));

        tankControl.getYawRange().setMin(BigDecimal.valueOf(-100));
        tankControl.getYawRange().setMax(BigDecimal.valueOf(100));
        tankControl.getYawRange().setCenter(BigDecimal.valueOf(0));
        tankControl.getYawRange().setOffset(BigDecimal.valueOf(0));

        tankControl.setThrottle(tankControl.getThrottleRange().getCenter());
        tankControl.setYaw(tankControl.getYawRange().getCenter());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void afterPropertiesSet() throws Exception {
        new Thread(tankControlUpdateTask, "TankControlUpdateThread").start();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void destroy() throws Exception {
    }

    private void tankControlUpdate() {
        TankControl tankControl = null;

        tankControlReadLock.lock();
        try {
            tankControl = this.tankControl.clone();
        } finally {
            tankControlReadLock.unlock();
        }

        BigDecimal t = tankControl.getThrottle();
        BigDecimal tMin = tankControl.getThrottleRange().getMin();
        BigDecimal tMax = tankControl.getThrottleRange().getMax();
        BigDecimal tCnt = tankControl.getThrottleRange().getCenter();

        BigDecimal y = tankControl.getYaw();
        BigDecimal yMin = tankControl.getYawRange().getMin();
        BigDecimal yMax = tankControl.getYawRange().getMax();
        BigDecimal yCnt = tankControl.getYawRange().getCenter();

        int throttle = 0;
        if (t.compareTo(tCnt) >= 0) {
            throttle = t.subtract(tCnt).divide(tMax.subtract(tCnt)).multiply(BigDecimal.valueOf(30000)).intValue();
        } else {
            throttle = t.subtract(tCnt).divide(tMin.subtract(tCnt)).multiply(BigDecimal.valueOf(-30000)).intValue();
        }

        int yaw = 0;
        if (y.compareTo(yCnt) >= 0) {
            yaw = y.subtract(yCnt).divide(yMax.subtract(yCnt)).multiply(BigDecimal.valueOf(30000)).intValue();
        } else {
            yaw = y.subtract(yCnt).divide(yMin.subtract(yCnt)).multiply(BigDecimal.valueOf(-30000)).intValue();
        }

        String cmd = "$AP0:" + yaw + "X" + throttle + "Y!";
        portService.sendCommandToPort(cmd);
    }

    private void triggerTankControlUpdate() {
        tankControlUpdateSemaphore.release();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankControl getTankControl() {
        tankControlReadLock.lock();
        try {
            return tankControl.clone();
        } finally {
            tankControlReadLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankControl setTankControl(TankControl tankControl) {
        tankControlWriteLock.lock();
        try {
            this.tankControl.setThrottle(tankControl.getThrottle());
            this.tankControl.setYaw(tankControl.getYaw());

            triggerTankControlUpdate();

            return new TankControl(this.tankControl);
        } finally {
            tankControlWriteLock.unlock();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankControlRange getTankControlRange() {
        tankControlReadLock.lock();
        try {
            return new TankControlRange(tankControl);
        } finally {
            tankControlReadLock.unlock();
        }
    }

}
