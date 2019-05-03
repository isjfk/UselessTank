/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.service;

import com.sap.cicn.tank.api.domain.CameraAngle;
import com.sap.cicn.tank.api.domain.CameraAngleRange;

/**
 * @author I311334
 */
public interface CameraPanTiltService {

    /**
     * Get camera angle range.
     *
     * @return camera angle range.
     */
    CameraAngleRange getCameraAngleRange();

    /**
     * Set camera angle range.
     *
     * @return camera angle range.
     */
    void setCameraAngleRange(CameraAngleRange cameraAngleRange);

    /**
     * Get current camera angle.
     *
     * @return current camera angle.
     */
    CameraAngle getCameraAngle();

    /**
     * Set camera angle.
     * <ul>
     * <li>angle.pan: Angle in horizontal direction. <0 to left, >0 to right.
     * <li>angle.tilt: Angle in vertical direction. <0 to down, >0 to up.
     * </ul>
     *
     * @param angle camera angle
     * @return actual camera angle set
     */
    CameraAngle setCameraAngle(CameraAngle angle);

    /**
     * Set camera to center angle.
     *
     * @return actual camera angle set
     */
    CameraAngle centerCamera();

}
