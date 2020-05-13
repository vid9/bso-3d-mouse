PROGRAM=sensors_math

#PROGRAM_SRC_FILES=./sensors_math.c

#EXTRA_COMPONENTS=extras/rboot-ota extras/i2c extras/mbedtls extras/httpd

EXTRA_COMPONENTS=extras/i2c

#EXTRA_CFLAGS=-DLWIP_HTTPD_CGI=1 -DLWIP_HTTPD_SSI=1 -I./fsdata

#LIBS += m # tole zakomentiraj 

include ~/esp-open-rtos/common.mk
