/**
 * Copyright (C) 1972-2017 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.common.exception;

/**
 * @author I311334
 */
public class BusinessException extends AbstractException {

    private static final long serialVersionUID = -370526948103747075L;

    /**
     * Construct instance.
     *
     * @param message
     */
    public BusinessException(Object... message) {
        super(message);
    }

    /**
     * Construct instance.
     *
     * @param cause
     * @param messages
     */
    public BusinessException(Throwable cause, Object... messages) {
        super(cause, messages);
    }

}
