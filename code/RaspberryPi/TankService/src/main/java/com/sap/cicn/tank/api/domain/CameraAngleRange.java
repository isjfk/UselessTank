/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

/**
 * @author I311334
 */
public class CameraAngleRange implements Cloneable {

    private ControlRange panRange = new ControlRange();
    private ControlRange tiltRange = new ControlRange();

    /**
     * Construct instance.
     */
    public CameraAngleRange() {
        super();
    }

    /**
     * Construct instance.
     *
     * @param cameraAngleRange
     */
    public CameraAngleRange(CameraAngleRange cameraAngleRange) {
        super();
        this.copyFrom(cameraAngleRange);
    }

    public CameraAngleRange copyFrom(CameraAngleRange that) {
        this.panRange = (that.panRange != null) ? that.panRange.clone() : new ControlRange();
        this.tiltRange = (that.tiltRange != null) ? that.tiltRange.clone() : new ControlRange();
        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngleRange clone() {
        try {
            CameraAngleRange that = (CameraAngleRange) super.clone();
            that.copyFrom(this);
            return that;
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @return the panRange
     */
    public ControlRange getPanRange() {
        return panRange;
    }

    /**
     * @param panRange the panRange to set
     */
    public void setPanRange(ControlRange panRange) {
        this.panRange = panRange;
    }

    /**
     * @return the tiltRange
     */
    public ControlRange getTiltRange() {
        return tiltRange;
    }

    /**
     * @param tiltRange the tiltRange to set
     */
    public void setTiltRange(ControlRange tiltRange) {
        this.tiltRange = tiltRange;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CameraAngleRange [panRange="
                + panRange
                + ", tiltRange="
                + tiltRange
                + ", super()="
                + super.toString()
                + "]";
    }

}
