echo SECR0=$(devmem2 0x4a320280|grep Value|awk '{print $6}')
echo SRSR0=$(devmem2 0x4a320200|grep Value|awk '{print $6}')
echo ESR0=$(devmem2 0x4a320300|grep Value|awk '{print $6}')
echo HIER=$(devmem2 0x4a321500|grep Value|awk '{print $6}')
echo CMR4=$(devmem2 0x4a320410|grep Value|awk '{print $6}')

