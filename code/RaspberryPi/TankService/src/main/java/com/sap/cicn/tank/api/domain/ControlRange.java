/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.api.domain;

import java.math.BigDecimal;

/**
 * @author I311334
 */
public class ControlRange implements Cloneable {

    private BigDecimal min;
    private BigDecimal max;

    private BigDecimal center;
    private BigDecimal offset;

    public BigDecimal limit(BigDecimal value) {
        if (value == null) {
            return null;
        }

        if ((min != null) && (value.compareTo(min) < 0)) {
            return min;
        }
        if ((max != null) && (value.compareTo(max) > 0)) {
            return max;
        }

        return value;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public ControlRange clone() {
        try {
            return (ControlRange) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @return the min
     */
    public BigDecimal getMin() {
        return min;
    }

    /**
     * @param min the min to set
     */
    public void setMin(BigDecimal min) {
        this.min = min;
    }

    /**
     * @return the max
     */
    public BigDecimal getMax() {
        return max;
    }

    /**
     * @param max the max to set
     */
    public void setMax(BigDecimal max) {
        this.max = max;
    }

    /**
     * @return the center
     */
    public BigDecimal getCenter() {
        return center;
    }

    /**
     * @param center the center to set
     */
    public void setCenter(BigDecimal center) {
        this.center = center;
    }

    /**
     * @return the offset
     */
    public BigDecimal getOffset() {
        return offset;
    }

    /**
     * @param offset the offset to set
     */
    public void setOffset(BigDecimal offset) {
        this.offset = offset;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "ControlRange [min="
                + min
                + ", max="
                + max
                + ", center="
                + center
                + ", offset="
                + offset
                + "]";
    }

}
