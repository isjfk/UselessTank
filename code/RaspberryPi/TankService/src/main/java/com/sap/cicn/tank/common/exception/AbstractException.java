/**
 * Copyright (C) 1972-2017 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.common.exception;

import org.apache.commons.lang3.StringUtils;

/**
 * @author I311334
 */
public abstract class AbstractException extends RuntimeException {

    private static final long serialVersionUID = -9143594636275398980L;

    /**
     * Construct instance.
     *
     * @param messages exception messages
     */
    public AbstractException(Object... messages) {
        super(StringUtils.join(messages));
    }

    /**
     * Construct instance.
     *
     * @param cause exception stack
     * @param messages exception messages
     */
    public AbstractException(Throwable cause, Object... messages) {
        super(StringUtils.join(messages), cause);
    }

}
