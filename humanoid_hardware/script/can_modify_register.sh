# Modify register values using busybox devmem
echo "Modifying register values..."
busybox devmem 0x0c303018 w 0xc458
busybox devmem 0x0c303010 w 0xc400
busybox devmem 0x0c303008 w 0xc458
busybox devmem 0x0c303000 w 0xc400