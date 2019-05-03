/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.hardware.i2c;

/**
 * @author I311334
 */
public class I2cAddress {

    private int busNumber;

    private int address;

    /**
     * Construct instance.
     *
     * @param busNumber
     * @param address
     */
    public I2cAddress(int busNumber, int address) {
        super();
        this.busNumber = busNumber;
        this.address = address;
    }

    /**
     * @return the busNumber
     */
    public int getBusNumber() {
        return busNumber;
    }

    /**
     * @param busNumber the busNumber to set
     */
    public void setBusNumber(int busNumber) {
        this.busNumber = busNumber;
    }

    /**
     * @return the address
     */
    public int getAddress() {
        return address;
    }

    /**
     * @param address the address to set
     */
    public void setAddress(int address) {
        this.address = address;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + address;
        result = prime * result + busNumber;
        return result;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (!(obj instanceof I2cAddress)) {
            return false;
        }
        I2cAddress other = (I2cAddress) obj;
        if (address != other.address) {
            return false;
        }
        if (busNumber != other.busNumber) {
            return false;
        }
        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "I2cAddress [busNumber=" + busNumber + ", address=" + address + "]";
    }

}
