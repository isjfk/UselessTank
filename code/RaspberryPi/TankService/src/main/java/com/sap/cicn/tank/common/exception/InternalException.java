/**
 * Copyright (C) 1972-2017 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.common.exception;

/**
 * @author I311334
 */
public class InternalException extends AbstractException {

    private static final long serialVersionUID = 5806411578322008832L;

    /**
     * Construct instance.
     *
     * @param message
     */
    public InternalException(Object... message) {
        super(message);
    }

    /**
     * Construct instance.
     *
     * @param cause
     * @param messages
     */
    public InternalException(Throwable cause, Object... messages) {
        super(cause, messages);
    }

}
