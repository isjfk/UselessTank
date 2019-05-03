/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

import java.math.BigDecimal;

/**
 * @author I311334
 */
public class CameraAngleRange implements Cloneable {

    private BigDecimal panMin;
    private BigDecimal panCenter;
    private BigDecimal panMax;
    private BigDecimal panOffset;

    private BigDecimal tiltMin;
    private BigDecimal tiltCenter;
    private BigDecimal tiltMax;
    private BigDecimal tiltOffset;

    public BigDecimal panInRange(BigDecimal pan) {
        if (pan == null) {
            return null;
        }
        if ((panMin != null) && (pan.compareTo(panMin) < 0)) {
            return panMin;
        }
        if ((panMax != null) && (pan.compareTo(panMax) > 0)) {
            return panMax;
        }
        return pan;
    }

    public BigDecimal tiltInRange(BigDecimal tilt) {
        if (tilt == null) {
            return null;
        }
        if ((tiltMin != null) && (tilt.compareTo(tiltMin) < 0)) {
            return tiltMin;
        }
        if ((tiltMax != null) && (tilt.compareTo(tiltMax) > 0)) {
            return tiltMax;
        }
        return tilt;
    }

    /**
     * @return the panMin
     */
    public BigDecimal getPanMin() {
        return panMin;
    }

    /**
     * @param panMin the panMin to set
     */
    public void setPanMin(BigDecimal panMin) {
        this.panMin = panMin;
    }

    /**
     * @return the panCenter
     */
    public BigDecimal getPanCenter() {
        return panCenter;
    }

    /**
     * @param panCenter the panCenter to set
     */
    public void setPanCenter(BigDecimal panCenter) {
        this.panCenter = panCenter;
    }

    /**
     * @return the panMax
     */
    public BigDecimal getPanMax() {
        return panMax;
    }

    /**
     * @param panMax the panMax to set
     */
    public void setPanMax(BigDecimal panMax) {
        this.panMax = panMax;
    }

    /**
     * @return the panOffset
     */
    public BigDecimal getPanOffset() {
        return panOffset;
    }

    /**
     * @param panOffset the panOffset to set
     */
    public void setPanOffset(BigDecimal panOffset) {
        this.panOffset = panOffset;
    }

    /**
     * @return the tiltMin
     */
    public BigDecimal getTiltMin() {
        return tiltMin;
    }

    /**
     * @param tiltMin the tiltMin to set
     */
    public void setTiltMin(BigDecimal tiltMin) {
        this.tiltMin = tiltMin;
    }

    /**
     * @return the tiltCenter
     */
    public BigDecimal getTiltCenter() {
        return tiltCenter;
    }

    /**
     * @param tiltCenter the tiltCenter to set
     */
    public void setTiltCenter(BigDecimal tiltCenter) {
        this.tiltCenter = tiltCenter;
    }

    /**
     * @return the tiltMax
     */
    public BigDecimal getTiltMax() {
        return tiltMax;
    }

    /**
     * @param tiltMax the tiltMax to set
     */
    public void setTiltMax(BigDecimal tiltMax) {
        this.tiltMax = tiltMax;
    }

    /**
     * @return the tiltOffset
     */
    public BigDecimal getTiltOffset() {
        return tiltOffset;
    }

    /**
     * @param tiltOffset the tiltOffset to set
     */
    public void setTiltOffset(BigDecimal tiltOffset) {
        this.tiltOffset = tiltOffset;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public CameraAngleRange clone() {
        try {
            return (CameraAngleRange) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CameraAngleRange [panMin="
                + panMin
                + ", panCenter="
                + panCenter
                + ", panMax="
                + panMax
                + ", panOffset="
                + panOffset
                + ", tiltMin="
                + tiltMin
                + ", tiltCenter="
                + tiltCenter
                + ", tiltMax="
                + tiltMax
                + ", tiltOffset="
                + tiltOffset
                + "]";
    }

}
