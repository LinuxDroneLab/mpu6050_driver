#echo 'changing opt ...' $(./devmem2 0x49004000 w 0x30E000 > /dev/null)
echo 'EDMA'
echo 'enabling event (EESR) ...' $(devmem2 0x49001030 w 0x0002 > /dev/null)
echo 'enabling interrupts (IESR) ...' $(devmem2 0x49001060 w 0x0002 > /dev/null)

#echo 'ECAP'
#echo 'stop ECAP ECCTL2 to 00CE .....' $(devmem2 0x4830012A h 0x00CE > /dev/null)
#echo 'reset interrupts ECCLR .......' $(devmem2 0x48300130 h 0xFFFF > /dev/null)
#echo 'enabling ECEINT 2 ............' $(devmem2 0x4830012C h 0x2 > /dev/null)
#echo 'setting ECCTL1 to C1EE .......' $(devmem2 0x48300128 h 0xC1EE > /dev/null)
#echo 'start ECAP ECCTL2 to 00DE ....' $(devmem2 0x4830012A h 0x00DE > /dev/null)
