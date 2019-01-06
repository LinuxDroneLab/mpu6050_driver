echo 'stop' > /sys/class/remoteproc/remoteproc1/state
echo 'stop' > /sys/class/remoteproc/remoteproc2/state
cp pru-components-bridge.out /lib/firmware/pru-components-bridge-fw && \
echo "pru-components-bridge-fw" > /sys/class/remoteproc/remoteproc2/firmware && \
echo 'start' > /sys/class/remoteproc/remoteproc2/state && \
cp pru-rpmsg-bridge.out /lib/firmware/pru-rpmsg-bridge-fw && \
echo "pru-rpmsg-bridge-fw" > /sys/class/remoteproc/remoteproc1/firmware && \
echo 'start' > /sys/class/remoteproc/remoteproc1/state 

