/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.util;

import com.sap.cicn.tank.common.exception.InternalException;

/**
 * @author I311334
 */
public class SystemUtils {

    public static void sleepAtLeast(long millis) {
        long start = System.currentTimeMillis();
        long elapse = 0;
        try {
            while (millis > elapse) {
                Thread.sleep(millis - elapse);

                elapse = System.currentTimeMillis() - start;
            }
        } catch (Exception e) {
            throw new InternalException(e, "Sleep interrupted.");
        }
    }

}
