VERSION 0.7
FROM debian:bookworm-20240110
RUN apt-get update -y
WORKDIR /RobotCode2024/vision

xx:
    FROM tonistiigi/xx:1.3.0
    SAVE ARTIFACT /usr/bin/*

py-build:
    COPY +xx/* /usr/bin
    RUN apt-get install -y wget build-essential cmake libffi-dev libssl-dev zlib1g-dev
    RUN wget https://www.python.org/ftp/python/3.10.13/Python-3.10.13.tgz
    RUN tar -zvxf Python-3.10.13.tgz
    RUN cp -r Python-3.10.13 Python-3.10.13-host
    WORKDIR Python-3.10.13
    RUN ./configure --prefix=/python3-build
    RUN make -j$(nproc)
    RUN make install
    SAVE IMAGE --cache-hint

py-host:
    FROM +py-build
    WORKDIR /RobotCode2024/vision/Python-3.10.13-host
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get install -y xx-c-essentials xx-cxx-essentials libffi-dev libssl-dev zlib1g-dev pkg-config
    RUN apt-get install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binutils-aarch64-linux-gnu
    RUN PATH=/python3-build/bin:$PATH ./configure --prefix=/python3-host --host=aarch64-linux-gnu --build=$(gcc -print-multiarch) --without-ensurepip --enable-shared --enable-optimizations --with-lto ac_cv_buggy_getaddrinfo=no ac_cv_file__dev_ptmx=yes ac_cv_file__dev_ptc=no
    RUN PATH=/python3-build/bin:$PATH make -j$(nproc)
    RUN PATH=/python3-build/bin:$PATH make install
    SAVE ARTIFACT /python3-host
    SAVE IMAGE --cache-hint

py-deps:
    FROM +py-host
    WORKDIR /RobotCode2024/vision
    RUN /python3-build/bin/pip3 install crossenv
    RUN /python3-build/bin/python3 -m crossenv /python3-host/bin/python3 cross_venv
    ENV VIRTUAL_ENV=/RobotCode2024/vision/cross_venv/cross
    ENV PATH="/RobotCode2024/vision/cross_venv/bin:$VIRTUAL_ENV/bin:$PATH"
    ENV MAKEFLAGS="-j$(nproc)"
    RUN apt-get update -y && apt-get install -y dh-autoreconf
    # If gfortran-aarch64-linux-gnu does not exist, we are on arm64, so just install gfortran
    RUN apt-get update -y && (apt-get install -y gfortran-aarch64-linux-gnu || apt-get install -y gfortran)
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y libopenblas-dev libjpeg-dev
    RUN wget https://files.pythonhosted.org/packages/a4/9b/027bec52c633f6556dba6b722d9a0befb40498b9ceddd29cbe67a45a127c/numpy-1.24.4.tar.gz
    RUN tar -zvxf numpy-1.24.4.tar.gz
    COPY numpy-aarch64-linux-gnu-site.cfg numpy-1.24.4/site.cfg
    RUN pip3 install -v ./numpy-1.24.4
    RUN pip3 install --find-links https://tortall.net/~robotpy/wheels/2023/raspbian pyntcore
    RUN pip3 install --find-links https://tortall.net/~robotpy/wheels/2023/raspbian robotpy-wpimath==2023.4.3.1
    RUN LDFLAGS="-L/usr/lib/aarch64-linux-gnu" CFLAGS="-I/usr/include/aarch64-linux-gnu" pip3 install -v pillow
    SAVE ARTIFACT /RobotCode2024/vision/cross_venv/cross
    SAVE IMAGE --cache-hint

opencv:
    FROM +py-deps
    WORKDIR /RobotCode2024/vision
    # Broken into multiple calls because we are seeing network issues when downloading all at once in Docker
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libtbbmalloc2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-dev gfortran openexr libatlas-base-dev
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y libgstreamer1.0-dev
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y libgstreamer-plugins-bad1.0-dev  gstreamer1.0-plugins-bad
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y gstreamer1.0-plugins-good
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y gstreamer1.0-plugins-ugly
    RUN TARGETPLATFORM=linux/arm64 xx-apt-get clean -y && TARGETPLATFORM=linux/arm64 xx-apt-get update -y && TARGETPLATFORM=linux/arm64 xx-apt-get install -y gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-gl
    RUN wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.6.0.tar.gz
    RUN wget -O opencv_contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/4.6.0.tar.gz
    RUN tar -zvxf opencv.tar.gz
    RUN tar -zvxf opencv_contrib.tar.gz
    WORKDIR opencv-4.6.0
    RUN mkdir build
    WORKDIR build
    RUN apt-get update -y && apt-get install -y clang
    ENV PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig
    ENV PKG_CONFIG_LIBDIR=/usr/lib/aarch64-linux-gnu/pkgconfig
    RUN cmake -DCMAKE_INSTALL_PREFIX=installroot -DCMAKE_TOOLCHAIN_FILE=/RobotCode2024/vision/opencv-4.6.0/platforms/linux/aarch64-gnu.toolchain.cmake -DWITH_GSTREAMER=ON -DWITH_FFMPEG=OFF -DPYTHON3_EXECUTABLE="/python3-build/bin/python3" -DPYTHON3_LIBRARIES="/python3-host/lib/libpython3.10.so" -DPYTHON3_NUMPY_INCLUDE_DIRS="/RobotCode2024/vision/cross_venv/cross/lib/python3.10/site-packages/numpy/core/include" -DPYTHON3_INCLUDE_PATH="/python3-host/include/python3.10" -DPYTHON3_CVPY_SUFFIX=".cpython-310-aarch64-linux-gnu.so" -D BUILD_NEW_PYTHON_SUPPORT=ON -D BUILD_opencv_python3=ON -D HAVE_opencv_python3=ON -D OPENCV_EXTRA_MODULES_PATH=/RobotCode2024/vision/opencv_contrib-4.6.0/modules -DBUILD_LIST=aruco,python3,videoio -D ENABLE_LTO=ON ..
    RUN make -j$(nproc)
    RUN make install
    SAVE ARTIFACT installroot/lib/*

sysroot-libs:
    FROM +opencv
    SAVE ARTIFACT --symlink-no-follow /usr/lib/aarch64-linux-gnu

sysroot-openblas-alternatives:
    FROM +opencv
    SAVE ARTIFACT --symlink-no-follow /etc/alternatives/libopenblas*

gst-bin:
    FROM +opencv
    SAVE ARTIFACT --symlink-no-follow /usr/bin/gst*

ns-image:
    FROM --platform=linux/arm64 debian:bookworm-20240110
    COPY --symlink-no-follow +sysroot-openblas-alternatives/* /etc/alternatives
    COPY --symlink-no-follow +sysroot-libs/* /usr/lib/aarch64-linux-gnu
    COPY --symlink-no-follow +gst-bin/* /usr/bin/
    COPY --dir +py-host/* /python3-host
    COPY +py-deps/cross/lib/python3.10/site-packages /python3-host/lib/python3.10/site-packages
    COPY +opencv/*.so* /usr/lib/aarch64-linux-gnu/
    COPY +opencv/python3.10/site-packages /python3-host/lib/python3.10/site-packages/
    ENV LD_LIBRARY_PATH=/python3-host/lib
    ENV PATH=/python3-host/bin:$PATH
    COPY --dir northstar /RobotCode2024/vision/northstar
    SAVE IMAGE littletonrobotics/northstar:latest