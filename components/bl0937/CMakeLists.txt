set(srcs "bl0937.c")              # BL0937 Driver

set(incs ".")

set(priv_req driver esp_timer)

idf_component_register(SRCS ${srcs}
    INCLUDE_DIRS ${incs}
    PRIV_REQUIRES ${priv_req}
    )
