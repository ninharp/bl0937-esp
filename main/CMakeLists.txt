set(srcs "main.c")              # Main Application

set(priv_req bl0937 driver esp_timer)

set(incs ".")

idf_component_register(SRCS ${srcs}
    INCLUDE_DIRS ${incs}
    PRIV_REQUIRES ${priv_req}
    )