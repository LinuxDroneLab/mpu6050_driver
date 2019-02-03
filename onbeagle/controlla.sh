echo EDMA
echo 'enabled events (EER) ......' $(devmem2 0x49001020|grep Value|awk '{print $6}')
echo 'enabled interrupts (IER) ..' $(devmem2 0x49001050 |grep Value|awk '{print $6}')
echo TPCC_CLK=$(devmem2 0x44E000BC | grep Value|awk '{print $6}')
echo TPTC0_CLK=$(devmem2 0x44E00024 | grep Value|awk '{print $6}')
echo
echo EDMA PaRAM
echo OPT=$(devmem2 0x49004000 | grep Value|awk '{print $6}')
echo SRC=$(devmem2 0x49004004 | grep Value|awk '{print $6}')
echo ACNT=$(devmem2 0x49004008 h | grep Value|awk '{print $6}')
echo BCNT=$(devmem2 0x4900400A h | grep Value|awk '{print $6}')
echo DST=$(devmem2 0x4900400C | grep Value|awk '{print $6}')
echo
echo EDMA Settings
echo IPR=$(devmem2 0x49001068|grep Value|awk '{print $6}')
echo EMR=$(devmem2 0x49000300 | grep Value|awk '{print $6}')
echo ER=$(devmem2 0x49001000 | grep Value|awk '{print $6}')
echo ERH=$(devmem2 0x49001004 | grep Value|awk '{print $6}')
echo EER=$(devmem2 0x49001020 | grep Value|awk '{print $6}')
echo EERH=$(devmem2 0x49001024 | grep Value|awk '{print $6}')
echo IER=$(devmem2 0x49001050 | grep Value|awk '{print $6}')
echo IERH=$(devmem2 0x49001054 | grep Value|awk '{print $6}')
echo IEVAL=$(devmem2 0x49001078 | grep Value|awk '{print $6}')
echo CER=$(devmem2 0x49001018 | grep Value|awk '{print $6}')
echo MPFAR=$(devmem2 0x49000800 | grep Value|awk '{print $6}')
echo MPFSR=$(devmem2 0x49000804 | grep Value|awk '{print $6}')
echo DRAE1=$(devmem2 0x49000348 | grep Value|awk '{print $6}')
echo MPPAG=$(devmem2 0x4900080C | grep Value|awk '{print $6}')
echo MPPA0=$(devmem2 0x49000810 | grep Value|awk '{print $6}')
echo MPPA1=$(devmem2 0x49000814 | grep Value|awk '{print $6}')
echo MPPA2=$(devmem2 0x49000818 | grep Value|awk '{print $6}')
echo MPPA3=$(devmem2 0x4900081C | grep Value|awk '{print $6}')
echo MPPA4=$(devmem2 0x49000820 | grep Value|awk '{print $6}')
echo MPPA5=$(devmem2 0x49000824 | grep Value|awk '{print $6}')
echo MPPA6=$(devmem2 0x49000828 | grep Value|awk '{print $6}')
echo MPPA7=$(devmem2 0x4900082C | grep Value|awk '{print $6}')
echo
echo PRU INTC
echo SECR0=$(devmem2 0x4a320280|grep Value|awk '{print $6}')
echo SRSR0=$(devmem2 0x4a320200|grep Value|awk '{print $6}')
echo ESR0=$(devmem2 0x4a320300|grep Value|awk '{print $6}')
echo HIER=$(devmem2 0x4a321500|grep Value|awk '{print $6}')
echo CMR4=$(devmem2 0x4a320410|grep Value|awk '{print $6}')
echo CMR3=$(devmem2 0x4a32040C | grep Value|awk '{print $6}')
echo HMR2=$(devmem2 0x4a320808|grep Value|awk '{print $6}')
echo
echo PRU MEMORY
echo 'buffer cap1-4 ..............' $(devmem2 0x4a302000 |grep Value|awk '{print $6}')
echo
echo ECAP
echo 'ECAP COUNTER ...............' $(devmem2 0x4a330000 |grep Value|awk '{print $6}')
echo 'ECAP ECAP1 .................' $(devmem2 0x4a330008 |grep Value|awk '{print $6}')
echo 'ECAP ECAP2 .................' $(devmem2 0x4a33000C |grep Value|awk '{print $6}')
echo 'ECAP ECAP3 .................' $(devmem2 0x4a330010 |grep Value|awk '{print $6}')
echo 'ECAP ECAP4 .................' $(devmem2 0x4a330014 |grep Value|awk '{print $6}')
echo 'ECAP ECEINT ................' $(devmem2 0x4a33002C h |grep Value|awk '{print $6}')
echo 'ECAP ECFLG .................' $(devmem2 0x4a33002E h |grep Value|awk '{print $6}')
echo 'ECAP ECCTL1 ................' $(devmem2 0x4a330028 h |grep Value|awk '{print $6}')
echo 'ECAP ECCTL2 ................' $(devmem2 0x4a33002A h |grep Value|awk '{print $6}')
echo

