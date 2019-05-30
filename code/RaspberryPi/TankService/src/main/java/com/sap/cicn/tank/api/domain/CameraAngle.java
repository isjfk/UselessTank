/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

import java.math.BigDecimal;

/**
 * @author I311334
 */
public class CameraAngle extends CameraAngleRange {

    private BigDecimal pan;
    private BigDecimal tilt;

    /**
     * Construct instance.
     */
    public CameraAngle() {
        super();
    }

    /**
     * Construct instance.
     *
     * @param cameraAngle
     */
    public CameraAngle(CameraAngle cameraAngle) {
        super(cameraAngle);
        this.copyFrom(cameraAngle);
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

    public CameraAngle copyFrom(CameraAngle that) {
        super.copyFrom(that);

        this.pan = this.getPanRange().limit(that.pan);
        this.tilt = this.getPanRange().limit(that.tilt);

        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngle clone() {
        CameraAngle that = (CameraAngle) super.clone();
        return that;
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
        this.pan = getPanRange().limit(pan);
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
        this.tilt = getTiltRange().limit(tilt);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CameraAngle [pan=" + pan + ", tilt=" + tilt + ", super()=" + super.toString() + "]";
    }

}
