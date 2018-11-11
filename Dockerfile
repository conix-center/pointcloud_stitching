FROM ubuntu:18.04


LABEL maintainer "Artur Balanuta"

MAINTAINER "artur@cmu.edu"

ENV work_dir /root

WORKDIR ${work_dir}



RUN	apt-get update && \
	apt-get install -y htop nano vim git wget aria2 build-essential cmake software-properties-common
	
RUN	apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || \
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

RUN	add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

RUN	apt-get update && \
	apt-get install -y librealsense2 librealsense2-utils librealsense2-dev librealsense2-dbg

RUN	DEBIAN_FRONTEND=noninteractive apt-get install -y libpcl-dev

RUN aria2c -x16 --summary-interval=1 "https://github.com/google/snappy/archive/1.1.7.tar.gz" && \
	tar -xvf 1.1.7.tar.gz && \
	mv snappy-1.1.7 snappy && \
	rm 1.1.7.tar.gz

#VOLUME /root

COPY CMakeLists.txt ${work_dir}
COPY calibration ${work_dir}/calibration
COPY src ${work_dir}/src
COPY samples ${work_dir}/samples
