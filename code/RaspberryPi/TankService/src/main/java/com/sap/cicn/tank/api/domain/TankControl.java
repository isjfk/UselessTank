/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

import java.math.BigDecimal;

/**
 * @author I311334
 */
public class TankControl extends TankControlRange {

    private BigDecimal throttle;
    private BigDecimal yaw;

    /**
     * Construct instance.
     *
     */
    public TankControl() {
        super();
    }

    /**
     * Construct instance.
     *
     * @param tankControl
     */
    public TankControl(TankControl tankControl) {
        super(tankControl);
        this.copyFrom(tankControl);
    }

    /**
     * Construct instance.
     *
     * @param throttle
     * @param yaw
     */
    public TankControl(BigDecimal throttle, BigDecimal yaw) {
        super();
        this.throttle = throttle;
        this.yaw = yaw;
    }

    public TankControl copyFrom(TankControl that) {
        super.copyFrom(that);

        this.throttle = this.getThrottleRange().limit(that.throttle);
        this.yaw = this.getYawRange().limit(that.yaw);

        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankControl clone() {
        TankControl that = (TankControl) super.clone();
        return that;
    }

    /**
     * @return the throttle
     */
    public BigDecimal getThrottle() {
        return throttle;
    }

    /**
     * @param throttle the throttle to set
     */
    public void setThrottle(BigDecimal throttle) {
        this.throttle = getThrottleRange().limit(throttle);
    }

    /**
     * @return the yaw
     */
    public BigDecimal getYaw() {
        return yaw;
    }

    /**
     * @param yaw the yaw to set
     */
    public void setYaw(BigDecimal yaw) {
        this.yaw = getYawRange().limit(yaw);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "TankControl [throttle=" + throttle + ", yaw=" + yaw + ", super()=" + super.toString() + "]";
    }

}
