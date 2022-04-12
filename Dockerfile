FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
RUN sed -i s:/archive.ubuntu.com:/mirrors.tuna.tsinghua.edu.cn/ubuntu:g /etc/apt/sources.list
RUN cat /etc/apt/sources.list
RUN apt-get clean
RUN apt-get -y update --fix-missing
# COPY ./keyboard /etc /default /keyboard
RUN apt install cmake gcc g++ sudo -y
RUN apt install libpcl-dev -y
ADD  ./ /root/plane/
RUN rm -rf /root/plane/build/*