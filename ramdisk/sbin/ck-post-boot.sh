#!/system/bin/sh

#
# replace (u)random with erandom
#
rm -f /dev/urandom
ln -s /dev/erandom /dev/urandom
rm -f /dev/random
ln -s /dev/erandom /dev/random

[ -f "/system/xbin/daemonsu" ] && /system/xbin/daemonsu --auto-daemon &

setenforce 0
pm disable com.sec.knox.seandroid

/system/xbin/busybox run-parts /system/etc/init.d
/system/xbin/busybox run-parts /sbin/init.d
