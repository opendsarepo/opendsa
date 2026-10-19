cd ./linux-kernel/bin
sudo insmod qdma-pf.ko
echo 8 > /sys/bus/pci/devices/0000\:18\:00.0/qdma/qmax
./dma-ctl qdma18000 q add idx 0 mode mm dir bi
./dma-ctl qdma18000 q start idx 0 dir bi

./dma-ctl qdma18000 q add idx 1 mode mm dir bi
./dma-ctl qdma18000 q start idx 1 dir bi

./dma-ctl qdma18000 q add idx 2 mode mm dir bi
./dma-ctl qdma18000 q start idx 2 dir bi

./dma-ctl qdma18000 q add idx 3 mode mm dir bi
./dma-ctl qdma18000 q start idx 3 dir bi




./dma-ctl qdma18000 q add idx 4 mode mm dir bi
./dma-ctl qdma18000 q start idx 4 dir bi

./dma-ctl qdma18000 q add idx 5 mode mm dir bi
./dma-ctl qdma18000 q start idx 5 dir bi

./dma-ctl qdma18000 q add idx 6 mode mm dir bi
./dma-ctl qdma18000 q start idx 6 dir bi

./dma-ctl qdma18000 q add idx 7 mode mm dir bi
./dma-ctl qdma18000 q start idx 7 dir bi

