#### 自己编写的framebuffer驱动，cat file > /dev/fb1可以看到花屏
#### 需要先insmod cfbfillrect.ko,cfbcopyarea.ko,cfbimgblt.ko,为了能看到显示还需要加载友善的一线触摸驱动来点亮背光
#### 对于framebuffer驱动优先移植，不建议自己写，毕竟官方的更加稳定，此处纯粹为了学习
