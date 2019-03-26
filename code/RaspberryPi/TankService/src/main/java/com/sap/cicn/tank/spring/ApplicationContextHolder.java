package com.sap.cicn.tank.spring;

import org.springframework.beans.BeansException;
import org.springframework.context.ApplicationContext;
import org.springframework.context.ApplicationContextAware;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;

/**
 * Created by i065037 on 2019/3/26.
 */
public class ApplicationContextHolder implements ApplicationContextAware {
    private static ApplicationContext context;

    @Override
    public void setApplicationContext(ApplicationContext context) throws BeansException {
        ApplicationContextHolder.context = context;
    }

    public static ApplicationContext getApplicationContext() {
        return context;
    }

    public static <T> T getBean(Class<T> clazz) {
        if (clazz == null) {
            return null;
        }

        return context == null ? null : context.getBean(clazz);
    }

    public static <T> Collection<T> getBeans(Class<T> clazz) {
        Map<String, T> beansMap = context.getBeansOfType(clazz);

        Iterator<Map.Entry<String, T>> iter = beansMap.entrySet().iterator();
        while (iter.hasNext()) {
            Map.Entry<String, T> entry = iter.next();
            if (entry.getKey().startsWith("scopedTarget")) {
                iter.remove();
            }
        }

        return beansMap.values();
    }

    public static <T> T getBean(String name, Class<T> clazz) {
        return context.getBean(name, clazz);
    }

    @SuppressWarnings("unchecked")
    public static <T> T getBean(String name, Object... args) {
        return (T) context.getBean(name, args);
    }

    public static Object getBean(String name) {
        return context.getBean(name);
    }

    public static boolean containsBean(String name) {
        return context.containsBean(name);
    }
}
