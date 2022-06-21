
# 将测试文件放在宿主机器 /task6 文件下
sudo mkdir -p /task6
sudo mkdir -p /task6_result/spirt
sudo rm /task6_result/spirt/*
<!-- sudo cp *.pcd /task6 -->
# build docker 构建镜像 
sudo docker build -t pl:v2 .
# 或者导入镜像
sudo docker load -i pl_v2.tar
# run 运行容器
sudo docker run --rm -it  -v /task6:/task6 -v /task6_result/spirt:/task6_result/spirt pl:v2 /usr/bin/bash /root/plane/run.sh
sudo docker run  -it  -v /task6:/task6 -v /task6_result/spirt:/task6_result/spirt pl:v2 /usr/bin/bash /root/plane/run.sh
