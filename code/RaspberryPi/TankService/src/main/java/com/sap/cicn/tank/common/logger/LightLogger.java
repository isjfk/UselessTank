package com.sap.cicn.tank.common.logger;

import org.apache.commons.lang3.StringUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.aop.support.AopUtils;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

/**
 * Created by i065037 on 2019/3/26.
 */
public class LightLogger {
    private static final int FIRST_LINE_MAX_LENGTH_DEFAULT = 250;
    private static int firstLineMaxLength = FIRST_LINE_MAX_LENGTH_DEFAULT;

    private static Map<String, LightLogger> loggerCacheMap = new ConcurrentHashMap<>();

    static {
        Thread lightLoggerStatThread = new Thread("LightLoggerCacheStatistics") {
            /* (non-Javadoc)
             * @see java.lang.Thread#run()
             */
            @Override
            public void run() {
                LightLogger log = LightLogger.getLogger(LightLogger.class);
                try {
                    while (true) {
                        log.info("LightLogger cache size: ", loggerCacheMap.size());
                        Thread.sleep(TimeUnit.MINUTES.toMillis(10));
                    }
                } catch (Exception e) {
                    log.error(e, "LightLogger cache statistics exit with exception.");
                }
            }
        };
        lightLoggerStatThread.setDaemon(true);
        lightLoggerStatThread.start();
    }

    private Logger log;

    /**
     * Construct instance.
     *
     * @param log
     */
    public LightLogger(Logger log) {
        super();
        this.log = log;
    }

    /**
     * Get logger for object.
     *
     * @param obj object
     * @return logger
     */
    public static LightLogger getLogger(Object obj) {
        return getLogger((obj != null) ? AopUtils.getTargetClass(obj) : LightLogger.class);
    }

    /**
     * Get logger for type.
     *
     * @param clazz class type
     * @return logger
     */
    public static LightLogger getLogger(Class<?> clazz) {
        Class<?> logClazz = (clazz != null) ? clazz : LightLogger.class;
        String logClazzName = logClazz.getCanonicalName();
        if (StringUtils.isBlank(logClazzName)) {
            logClazzName = logClazz.getName();
        }

        LightLogger logger = loggerCacheMap.get(logClazzName);
        if (logger == null) {
            logger = new LightLogger(LoggerFactory.getLogger(logClazz));
            loggerCacheMap.put(logClazzName, logger);
        }

        return logger;
    }

    /**
     * @return the firstLineMaxLength
     */
    protected static int getFirstLineMaxLength() {
        return firstLineMaxLength;
    }

    /**
     * @param firstLineMaxLength the firstLineMaxLength to set
     */
    protected static void setFirstLineMaxLength(int firstLineMaxLength) {
        LightLogger.firstLineMaxLength = firstLineMaxLength;
    }

    /**
     * Write sync trace log.
     *
     * @param messages trace message
     */
    public void debug(Object... messages) {
        if (log.isDebugEnabled()) {
            log.debug(genLog(messages));
        }
    }

    /**
     * Write sync trace log.
     *
     * @param message trace message
     */
    public void debug(String message) {
        if (log.isDebugEnabled()) {
            log.debug(genLog(message));
        }
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param message trace message
     */
    public void debug(Throwable e, String message) {
        if (log.isDebugEnabled()) {
            log.debug(genLog(message), e);
        }
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param messages trace message
     */
    public void debug(Throwable e, Object... messages) {
        if (log.isDebugEnabled()) {
            log.debug(genLog(messages), e);
        }
    }

    /**
     * Write sync trace log.
     *
     * @param messages trace message
     */
    public void info(Object... messages) {
        if (log.isInfoEnabled()) {
            log.info(genLog(messages));
        }
    }

    /**
     * Write sync trace log.
     *
     * @param message trace message
     */
    public void info(String message) {
        if (log.isInfoEnabled()) {
            log.info(genLog(message));
        }
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param message trace message
     */
    public void info(Throwable e, String message) {
        if (log.isInfoEnabled()) {
            log.info(genLog(message), e);
        }
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param messages trace message
     */
    public void info(Throwable e, Object... messages) {
        if (log.isInfoEnabled()) {
            log.info(genLog(messages), e);
        }
    }

    /**
     * Write sync trace log.
     *
     * @param messages trace message
     */
    public void warn(Object... messages) {
        log.warn(genLog(messages));
    }

    /**
     * Write sync trace log.
     *
     * @param message trace message
     */
    public void warn(String message) {
        log.warn(genLog(message));
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param message trace message
     */
    public void warn(Throwable e, String message) {
        log.warn(genLog(message), e);
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param messages trace message
     */
    public void warn(Throwable e, Object... messages) {
        log.warn(genLog(messages), e);
    }

    /**
     * Write sync trace log.
     *
     * @param messages trace message
     */
    public void error(Object... messages) {
        log.error(genLog(messages));
    }

    /**
     * Write sync trace log.
     *
     * @param message trace message
     */
    public void error(String message) {
        log.error(genLog(message));
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param message trace message
     */
    public void error(Throwable e, String message) {
        log.error(genLog(message), e);
    }

    /**
     * Write sync trace log.
     *
     * @param e exception
     * @param messages trace message
     */
    public void error(Throwable e, Object... messages) {
        log.error(genLog(messages), e);
    }

    private String genLog(String message) {
        return handleFirstLineLengthLimit(message);
    }

    private String genLog(Object[] messages) {
        return handleFirstLineLengthLimit(StringUtils.join(messages));
    }

    private String handleFirstLineLengthLimit(String message) {
        if ((message == null) || (message.length() <= firstLineMaxLength)) {
            return message;
        }
        if (message.lastIndexOf('\n', firstLineMaxLength) >= 0) {
            return message;
        }

        int splitPos = searchSplitPosBySpace(message);
        if (splitPos >= 0) {
            return message.substring(0, splitPos) + System.lineSeparator() + message.substring(splitPos);
        } else {
            return message.substring(0, firstLineMaxLength) + System.lineSeparator() + "    " + message.substring(firstLineMaxLength);
        }
    }

    private int searchSplitPosBySpace(String message) {
        for (int i = firstLineMaxLength; i > 0; i--) {
            if (Character.isWhitespace(message.charAt(i))) {
                for (int j = i - 1; j >= 0; j--) {
                    if (!Character.isWhitespace(message.charAt(j))) {
                        return j + 1;
                    }
                }
                return -1;
            }
        }
        return -1;
    }
}
