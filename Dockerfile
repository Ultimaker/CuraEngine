FROM ubuntu:latest

RUN apt-get update && apt-get install -y \
  autoconf \
  cmake \
  g++ \
  git \
  libtool \
  python3-sip-dev

# Install protobuff
RUN git clone https://github.com/protocolbuffers/protobuf.git
WORKDIR /protobuf
RUN ./autogen.sh
RUN ./configure
RUN make
RUN make install
RUN ldconfig

# Install libArcus
WORKDIR /
RUN git clone https://github.com/Ultimaker/libArcus.git
RUN mkdir -p /libArcus/build
WORKDIR /libArcus/build
RUN cmake ..
RUN make
RUN make install

RUN mkdir -p /CuraEngine
WORKDIR /CuraEngine
COPY . .
RUN mkdir -p build
WORKDIR /CuraEngine/build
RUN cmake ..
RUN make

RUN ./CuraEngine help

