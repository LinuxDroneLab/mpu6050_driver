  cd /root/pid/ && \
  touch * && \
  make clean  && \
  make load && \
  ./install.sh && \
  sleep 5 && \
  cd /sys/bus/iio/devices/iio\:device1 && \
  cd scan_elements/ && \
  echo 1 > in_accel0_en && \
  cd ../buffer && \
  echo 1 > enable  && \
  tail -f /var/log/syslog

