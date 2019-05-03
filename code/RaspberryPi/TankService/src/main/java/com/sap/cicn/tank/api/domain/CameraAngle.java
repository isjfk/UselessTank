/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

import java.math.BigDecimal;

/**
 * @author I311334
 */
public class CameraAngle implements Cloneable {

    private BigDecimal pan;
    private BigDecimal tilt;

    private CameraAngleRange range;

    /**
     * Construct instance.
     */
    public CameraAngle() {
        super();
    }

    /**
     * Construct instance.
     *
     * @param pan
     * @param tilt
     */
    public CameraAngle(BigDecimal pan, BigDecimal tilt) {
        super();
        this.pan = pan;
        this.tilt = tilt;
    }

    /**
     * Construct instance.
     *
     * @param pan
     * @param tilt
     * @param range
     */
    public CameraAngle(BigDecimal pan, BigDecimal tilt, CameraAngleRange range) {
        super();
        if (range != null) {
            this.pan = range.panInRange(pan);
            this.tilt = range.tiltInRange(tilt);
            this.range = range;
        } else {
            this.pan = pan;
            this.tilt = tilt;
            this.range = range;
        }
    }

    /**
     * @return the pan
     */
    public BigDecimal getPan() {
        return pan;
    }

    /**
     * @param pan the pan to set
     */
    public void setPan(BigDecimal pan) {
        this.pan = pan;
    }

    /**
     * @return the tilt
     */
    public BigDecimal getTilt() {
        return tilt;
    }

    /**
     * @param tilt the tilt to set
     */
    public void setTilt(BigDecimal tilt) {
        this.tilt = tilt;
    }

    /**
     * @return the range
     */
    public CameraAngleRange getRange() {
        return range;
    }

    /**
     * @param range the range to set
     */
    public void setRange(CameraAngleRange range) {
        this.range = range;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle clone() {
        try {
            CameraAngle that = (CameraAngle) super.clone();
            that.setRange(this.range.clone());
            return that;
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CameraAngle [pan="
                + pan
                + ", tilt="
                + tilt
                + ", range="
                + range
                + "]";
    }

}
