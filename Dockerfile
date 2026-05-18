FROM ubuntu:22.04
 
ENV DEBIAN_FRONTEND=noninteractive
 
# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libsfml-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*
 
WORKDIR /app
 
COPY . .
 
RUN cmake -B build -S . && cmake --build build
 
CMD ["./build/HNMG"]