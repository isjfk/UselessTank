/**
 * Copyright (C) 1972-2017 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.common.exception;

/**
 * @author I311334
 */
public class RemoteException extends AbstractException {

    private static final long serialVersionUID = -9076703585925842181L;

    /**
     * Construct instance.
     *
     * @param message
     */
    public RemoteException(Object... message) {
        super(message);
    }

    /**
     * Construct instance.
     *
     * @param cause
     * @param messages
     */
    public RemoteException(Throwable cause, Object... messages) {
        super(cause, messages);
    }

}
