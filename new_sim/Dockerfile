# Use the official Ubuntu 22.04 LTS as the base image
FROM ubuntu:22.04

# Set environment variables to non-interactive to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential packages
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        sudo \
        network-manager \
        iproute2 \
        can-utils \
        build-essential \
        x11-apps \
        x11-xserver-utils \
        vim \
        wget \
        curl \
        && rm -rf /var/lib/apt/lists/*

# (Optional) Create a non-root user for better security
RUN useradd -m dockeruser && echo "dockeruser:docker" | chpasswd && adduser dockeruser sudo

# Set the working directory
WORKDIR /home/dockeruser

# Switch to the non-root user
USER dockeruser

# Set the default command to bash
CMD ["/bin/bash"]
