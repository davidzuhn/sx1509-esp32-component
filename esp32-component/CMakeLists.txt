set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS
    SX1509.cpp
   )

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES
    "I2C"
   )
set(COMPONENT_PRIV_REQUIRES )


register_component()
