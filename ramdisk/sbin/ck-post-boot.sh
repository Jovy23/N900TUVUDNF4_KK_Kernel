#!/system/bin/sh

[ -f "/system/xbin/daemonsu" ] && /system/xbin/daemonsu --auto-daemon &

setenforce 0
pm disable com.sec.knox.seandroid

/system/xbin/busybox run-parts /system/etc/init.d
/system/xbin/busybox run-parts /sbin/init.d
