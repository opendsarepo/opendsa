cd ./linux-kernel/bin
sudo insmod qdma-pf.ko
echo 8 > /sys/bus/pci/devices/0000\:18\:00.0/qdma/qmax
./dma-ctl qdma18000 q add idx 0 mode mm dir bi
./dma-ctl qdma18000 q start idx 0 dir bi
