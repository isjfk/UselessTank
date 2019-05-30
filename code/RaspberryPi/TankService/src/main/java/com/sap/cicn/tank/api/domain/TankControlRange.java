/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

/**
 * @author I311334
 */
public class TankControlRange implements Cloneable {

    private ControlRange throttleRange = new ControlRange();
    private ControlRange yawRange = new ControlRange();

    /**
     * Construct instance.
     */
    public TankControlRange() {
        super();
    }

    /**
     * Construct instance.
     *
     * @param tankControlRange
     */
    public TankControlRange(TankControlRange tankControlRange) {
        super();
        this.copyFrom(tankControlRange);
    }

    public TankControlRange copyFrom(TankControlRange that) {
        this.throttleRange = (that.throttleRange != null) ? that.throttleRange.clone() : new ControlRange();
        this.yawRange = (that.yawRange != null) ? that.yawRange.clone() : new ControlRange();
        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankControlRange clone() {
        try {
            TankControlRange that = (TankControlRange) super.clone();
            that.copyFrom(this);
            return that;
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @return the throttleRange
     */
    public ControlRange getThrottleRange() {
        return throttleRange;
    }

    /**
     * @param throttleRange the throttleRange to set
     */
    public void setThrottleRange(ControlRange throttleRange) {
        this.throttleRange = throttleRange;
    }

    /**
     * @return the yawRange
     */
    public ControlRange getYawRange() {
        return yawRange;
    }

    /**
     * @param yawRange the yawRange to set
     */
    public void setYawRange(ControlRange yawRange) {
        this.yawRange = yawRange;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "TankControlRange [throttleRange="
                + throttleRange
                + ", yawRange="
                + yawRange
                + "]";
    }

}
