PROGRAM=test

PROGRAM_SRC_FILES=./test.c

#EXTRA_COMPONENTS=extras/rboot-ota extras/i2c extras/mbedtls extras/httpd

EXTRA_COMPONENTS=extras/i2c

#EXTRA_CFLAGS=-DLWIP_HTTPD_CGI=1 -DLWIP_HTTPD_SSI=1 -I./fsdata


include ../../common.mk
