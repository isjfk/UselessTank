/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.service;

import com.sap.cicn.tank.api.domain.TankControl;
import com.sap.cicn.tank.api.domain.TankControlRange;

/**
 * @author I311334
 */
public interface TankControlService {

    /**
     * Get current tank control.
     *
     * @return current tank control
     */
    TankControl getTankControl();

    /**
     * Set tank control.
     *
     * @param tankControl tank control
     * @return actual tank control set
     */
    TankControl setTankControl(TankControl tankControl);

    /**
     * Get tank control range.
     *
     * @return tank control range
     */
    TankControlRange getTankControlRange();

}
